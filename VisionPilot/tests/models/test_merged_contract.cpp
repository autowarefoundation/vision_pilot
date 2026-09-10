#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <models/auto_drive.hpp>
#include <models/auto_steer.hpp>
#include <models/merged_contract.hpp>
#include <numeric>
#include <string>
#include <vector>

using visionpilot::models::MergedContract;
using visionpilot::models::resolve_contract_path;

namespace {

// The v7 contract emitted by openadkit's rewrite pipeline, verbatim from
// v7_frozen.onnx.contract.json.
const char* kV7Contract = R"({
 "attn_mode": "frozen",
 "passthrough": ["steer_height"],
 "head": {
  "output": "drive_head_raw",
  "alpha": [8.0, 512.0, 0.5],
  "map": [["drive_distance", "relu"],
          ["drive_curvature", "tanh"],
          ["drive_flag_logit", "none"]]
 },
 "speed": {
  "levels": [
   {"box": "speed_l15_box", "cls": "speed_l15_cls", "stride": 8,  "hw": [64, 128]},
   {"box": "speed_l16_box", "cls": "speed_l16_cls", "stride": 16, "hw": [32, 64]},
   {"box": "speed_l17_box", "cls": "speed_l17_cls", "stride": 32, "hw": [16, 32]}
  ],
  "note": "boxes_l = box*stride"
 },
 "steer_xp": {
  "logits": "steer_silu_41",
  "positions": 256,
  "div": 256.0,
  "note": "softmax(logits [64,256], axis=-1) @ arange(positions) / div"
 }
})";

// The v6 contract, still supported: lane comes through passthrough instead.
const char* kV6Contract = R"({
 "attn_mode": "frozen",
 "passthrough": ["steer_lane_value", "steer_height"],
 "head": {
  "output": "drive_head_raw",
  "alpha": [8.0, 512.0, 0.5],
  "map": [["drive_distance", "relu"],
          ["drive_curvature", "tanh"],
          ["drive_flag_logit", "none"]]
 },
 "speed": {
  "levels": [
   {"box": "speed_l15_box", "cls": "speed_l15_cls", "stride": 8, "hw": [64, 128]}
  ]
 }
})";

// A unique on-disk directory for one test, removed (recursively) when the
// test's scope ends, including on an ASSERT-triggered early return.
class ScopedTempDir
{
public:
    ScopedTempDir()
        : path_(std::filesystem::temp_directory_path() /
                ("visionpilot_merged_contract_test_" +
                 std::to_string(
                     std::chrono::steady_clock::now().time_since_epoch().count())))
    {
        std::filesystem::create_directories(path_);
    }

    ~ScopedTempDir() { std::filesystem::remove_all(path_); }

    const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};

void write_file(const std::filesystem::path& path, const std::string& content)
{
    std::ofstream out(path);
    out << content;
}

}  // namespace

TEST(MergedContract, ParsesV7Contract)
{
    const auto c = MergedContract::from_json_string(kV7Contract);

    EXPECT_EQ(c.attn_mode, "frozen");

    // v7 passes only the height vector through; lane is computed on the host.
    ASSERT_EQ(c.passthrough.size(), 1u);
    EXPECT_EQ(c.passthrough[0], "steer_height");

    ASSERT_TRUE(c.head.has_value());
    EXPECT_EQ(c.head->output, "drive_head_raw");
    ASSERT_EQ(c.head->alpha.size(), 3u);
    EXPECT_FLOAT_EQ(c.head->alpha[0], 8.0f);
    EXPECT_FLOAT_EQ(c.head->alpha[1], 512.0f);
    EXPECT_FLOAT_EQ(c.head->alpha[2], 0.5f);
    ASSERT_EQ(c.head->map.size(), 3u);
    EXPECT_EQ(c.head->map[0].output, "drive_distance");
    EXPECT_EQ(c.head->map[0].activation, "relu");
    EXPECT_EQ(c.head->map[1].activation, "tanh");
    EXPECT_EQ(c.head->map[2].activation, "none");

    ASSERT_TRUE(c.steer_xp.has_value());
    EXPECT_EQ(c.steer_xp->logits, "steer_silu_41");
    EXPECT_EQ(c.steer_xp->positions, 256);
    EXPECT_FLOAT_EQ(c.steer_xp->div, 256.0f);

    ASSERT_TRUE(c.speed.has_value());
    ASSERT_EQ(c.speed->levels.size(), 3u);
    EXPECT_EQ(c.speed->levels[0].box, "speed_l15_box");
    EXPECT_EQ(c.speed->levels[0].cls, "speed_l15_cls");
    EXPECT_EQ(c.speed->levels[0].stride, 8);
    EXPECT_EQ(c.speed->levels[0].h, 64);
    EXPECT_EQ(c.speed->levels[0].w, 128);
    EXPECT_EQ(c.speed->levels[2].stride, 32);
    EXPECT_EQ(c.speed->levels[2].h, 16);
    EXPECT_EQ(c.speed->levels[2].w, 32);
}

TEST(MergedContract, ParsesV6ContractWithoutSteerXp)
{
    const auto c = MergedContract::from_json_string(kV6Contract);
    ASSERT_EQ(c.passthrough.size(), 2u);
    EXPECT_EQ(c.passthrough[0], "steer_lane_value");
    EXPECT_FALSE(c.steer_xp.has_value());
    ASSERT_TRUE(c.head.has_value());
}

TEST(MergedContract, MinimalContractLeavesOptionalsEmpty)
{
    const auto c = MergedContract::from_json_string(R"({"attn_mode": "keep"})");
    EXPECT_EQ(c.attn_mode, "keep");
    EXPECT_TRUE(c.passthrough.empty());
    EXPECT_FALSE(c.head.has_value());
    EXPECT_FALSE(c.speed.has_value());
    EXPECT_FALSE(c.steer_xp.has_value());
}

TEST(MergedContract, RejectsMalformedJson)
{
    EXPECT_THROW(MergedContract::from_json_string("{ not json"),
                 std::runtime_error);
}

TEST(MergedContract, RejectsAlphaMapLengthMismatch)
{
    EXPECT_THROW(MergedContract::from_json_string(R"({
      "head": {"output": "drive_head_raw", "alpha": [1.0, 2.0],
               "map": [["drive_distance", "relu"]]}
    })"), std::runtime_error);
}

TEST(MergedContract, RejectsNonPositiveSteerXpPositions)
{
    EXPECT_THROW(MergedContract::from_json_string(R"({
      "steer_xp": {"logits": "x", "positions": 0, "div": 256.0}
    })"), std::runtime_error);
}

TEST(MergedContract, RejectsZeroSteerXpDivisor)
{
    EXPECT_THROW(MergedContract::from_json_string(R"({
      "steer_xp": {"logits": "x", "positions": 256, "div": 0.0}
    })"), std::runtime_error);
}

TEST(MergedContract, DetectsRewriteSignatureOutputs)
{
    using visionpilot::models::has_rewrite_signature_output;
    EXPECT_TRUE(has_rewrite_signature_output({"steer_height", "drive_head_raw"}));
    EXPECT_TRUE(has_rewrite_signature_output({"speed_l15_box", "speed_l15_cls"}));
    EXPECT_FALSE(has_rewrite_signature_output(
        {"steer_xp", "speed_output", "drive_distance"}));
}

TEST(MergedContract, SplitsRewriteSignatureIntoDriveAndSpeedHalves)
{
    using visionpilot::models::is_drive_head_raw_output;
    using visionpilot::models::is_speed_level_box_output;

    // The two halves answer independently: the drive marker is not a speed
    // marker and vice versa, which is what lets validate_contract_names()
    // demand a head rule and a speed rule separately.
    EXPECT_TRUE(is_drive_head_raw_output("drive_head_raw"));
    EXPECT_FALSE(is_speed_level_box_output("drive_head_raw"));

    EXPECT_TRUE(is_speed_level_box_output("speed_l15_box"));
    EXPECT_TRUE(is_speed_level_box_output("speed_l17_box"));
    EXPECT_FALSE(is_drive_head_raw_output("speed_l15_box"));

    // Neither marker: the plain-merged output names and the cls half of a
    // rewritten speed level.
    EXPECT_FALSE(is_speed_level_box_output("speed_l15_cls"));
    EXPECT_FALSE(is_speed_level_box_output("speed_output"));
    EXPECT_FALSE(is_drive_head_raw_output("drive_distance"));
}

TEST(ResolveContractPath, PrefersSidecarWhenOnlySidecarExists)
{
    ScopedTempDir dir;
    const auto model_path = (dir.path() / "model.onnx").string();
    const auto sidecar    = model_path + ".contract.json";
    write_file(sidecar, "{}");

    EXPECT_EQ(resolve_contract_path(model_path, ""), sidecar);
}

TEST(ResolveContractPath, FallsBackToArtifactsDirWhenSidecarIsAbsent)
{
    ScopedTempDir dir;
    const auto model_path    = (dir.path() / "model.onnx").string();
    const auto artifacts_dir = dir.path().string();
    const auto in_dir        = (dir.path() / "contract.json").string();
    write_file(in_dir, "{}");

    EXPECT_EQ(resolve_contract_path(model_path, artifacts_dir), in_dir);
}

TEST(ResolveContractPath, PrefersSidecarWhenBothExist)
{
    ScopedTempDir dir;
    const auto model_path    = (dir.path() / "model.onnx").string();
    const auto artifacts_dir = dir.path().string();
    const auto sidecar       = model_path + ".contract.json";
    write_file(sidecar, "{}");
    write_file((dir.path() / "contract.json").string(), "{}");

    EXPECT_EQ(resolve_contract_path(model_path, artifacts_dir), sidecar);
}

TEST(ResolveContractPath, ReturnsEmptyWhenNeitherExists)
{
    ScopedTempDir dir;
    const auto model_path    = (dir.path() / "model.onnx").string();
    const auto artifacts_dir = dir.path().string();

    EXPECT_EQ(resolve_contract_path(model_path, artifacts_dir), "");
}

TEST(ResolveContractPath, UsesArtifactsDirWhenModelPathIsEmpty)
{
    ScopedTempDir dir;
    const auto artifacts_dir = dir.path().string();
    const auto in_dir        = (dir.path() / "contract.json").string();
    write_file(in_dir, "{}");

    EXPECT_EQ(resolve_contract_path("", artifacts_dir), in_dir);
}

TEST(ResolveContractPath, ReturnsEmptyWhenBothArgumentsAreEmpty)
{
    EXPECT_EQ(resolve_contract_path("", ""), "");
}

// A contract is distributed under the sidecar name of the model it was
// rewritten from, and under the renesas provider there is no model path to
// hang that name off -- resolve_merged_target() passes an empty model_path and
// the file actually loaded is the compiled qdq_inserted_* one. So the
// distributed name has to be accepted inside the artifacts directory.
TEST(ResolveContractPath, AcceptsDistributedSidecarNameInsideArtifactsDir)
{
    ScopedTempDir dir;
    const auto artifacts_dir = dir.path().string();
    const auto distributed =
        (dir.path() / "v7_frozen.onnx.contract.json").string();
    write_file(distributed, "{}");

    EXPECT_EQ(resolve_contract_path("", artifacts_dir), distributed);
}

TEST(ResolveContractPath, PrefersPlainContractJsonOverDistributedName)
{
    ScopedTempDir dir;
    const auto artifacts_dir = dir.path().string();
    const auto plain = (dir.path() / "contract.json").string();
    write_file(plain, "{}");
    write_file((dir.path() / "v7_frozen.onnx.contract.json").string(), "{}");

    EXPECT_EQ(resolve_contract_path("", artifacts_dir), plain);
}

// Choosing one would silently decide which rewrite's host postprocessing runs,
// and nothing downstream can detect the wrong pick.
TEST(ResolveContractPath, RefusesTwoDistributedSidecarsAsAmbiguous)
{
    ScopedTempDir dir;
    const auto artifacts_dir = dir.path().string();
    write_file((dir.path() / "v6_frozen.onnx.contract.json").string(), "{}");
    write_file((dir.path() / "v7_frozen.onnx.contract.json").string(), "{}");

    EXPECT_THROW(resolve_contract_path("", artifacts_dir),
                 std::runtime_error);
}

TEST(ResolveContractPath, IgnoresUnrelatedFilesInArtifactsDir)
{
    ScopedTempDir dir;
    const auto artifacts_dir = dir.path().string();
    write_file((dir.path() / "manifest.json").string(), "{}");
    write_file((dir.path() / "contract.json.bak").string(), "{}");

    EXPECT_EQ(resolve_contract_path("", artifacts_dir), "");
}

TEST(ResolveContractPath, ModelPathSidecarStillWinsOverDistributedName)
{
    ScopedTempDir dir;
    const auto artifacts_dir = dir.path().string();
    const auto model_path    = (dir.path() / "model.onnx").string();
    const auto sidecar       = model_path + ".contract.json";
    write_file(sidecar, "{}");
    write_file((dir.path() / "v7_frozen.onnx.contract.json").string(), "{}");

    EXPECT_EQ(resolve_contract_path(model_path, artifacts_dir), sidecar);
}

TEST(ResolveContractPath, ReturnsEmptyWhenArtifactsDirDoesNotExist)
{
    EXPECT_EQ(resolve_contract_path("", "/nonexistent/artifacts/dir"), "");
}

TEST(MergedContractFromFile, ParsesSameAsFromJsonString)
{
    ScopedTempDir dir;
    const auto path = (dir.path() / "v7.contract.json").string();
    write_file(path, kV7Contract);

    const auto from_file   = MergedContract::from_file(path);
    const auto from_string = MergedContract::from_json_string(kV7Contract);

    EXPECT_EQ(from_file.attn_mode, from_string.attn_mode);
    EXPECT_EQ(from_file.passthrough, from_string.passthrough);

    ASSERT_TRUE(from_file.head.has_value());
    ASSERT_TRUE(from_string.head.has_value());
    EXPECT_EQ(from_file.head->output, from_string.head->output);
    EXPECT_EQ(from_file.head->alpha, from_string.head->alpha);

    ASSERT_TRUE(from_file.steer_xp.has_value());
    ASSERT_TRUE(from_string.steer_xp.has_value());
    EXPECT_EQ(from_file.steer_xp->logits, from_string.steer_xp->logits);

    ASSERT_TRUE(from_file.speed.has_value());
    ASSERT_TRUE(from_string.speed.has_value());
    EXPECT_EQ(from_file.speed->levels.size(), from_string.speed->levels.size());
}

TEST(MergedContractFromFile, ThrowsWhenPathDoesNotExist)
{
    ScopedTempDir dir;
    const auto path = (dir.path() / "does_not_exist.contract.json").string();

    EXPECT_THROW(MergedContract::from_file(path), std::runtime_error);
}

TEST(MergedContractFromFile, ThrowsOnMalformedJson)
{
    ScopedTempDir dir;
    const auto path = (dir.path() / "malformed.contract.json").string();
    write_file(path, "{ not json");

    EXPECT_THROW(MergedContract::from_file(path), std::runtime_error);
}

TEST(ApplyHead, DividesByAlphaAndAppliesActivations)
{
    using visionpilot::models::apply_head;
    const auto c = MergedContract::from_json_string(kV7Contract);
    ASSERT_TRUE(c.head.has_value());

    // raw = [dist_pre*8, curv_pre*512, flag_logit*0.5]
    // Chosen pre-activation values: dist 0.25, curv -0.002, flag 1.5
    const float raw[3] = {0.25f * 8.0f, -0.002f * 512.0f, 1.5f * 0.5f};

    visionpilot::models::AutoDriveOutput out;
    apply_head(*c.head, raw, 3, out);

    EXPECT_TRUE(out.valid);
    EXPECT_NEAR(out.dist_normalized, 0.25f, 1e-6f);          // relu(0.25)
    EXPECT_NEAR(out.curvature_raw, std::tanh(-0.002f), 1e-6f);
    EXPECT_NEAR(out.flag_prob, 1.f / (1.f + std::exp(-1.5f)), 1e-6f);
}

TEST(ApplyHead, ReluClampsNegativeDistance)
{
    using visionpilot::models::apply_head;
    const auto c = MergedContract::from_json_string(kV7Contract);
    const float raw[3] = {-1.0f * 8.0f, 0.0f, 0.0f};

    visionpilot::models::AutoDriveOutput out;
    apply_head(*c.head, raw, 3, out);
    EXPECT_FLOAT_EQ(out.dist_normalized, 0.0f);
}

TEST(ApplyHead, RejectsShortRawBuffer)
{
    using visionpilot::models::apply_head;
    const auto c = MergedContract::from_json_string(kV7Contract);
    const float raw[2] = {0.f, 0.f};
    visionpilot::models::AutoDriveOutput out;
    EXPECT_THROW(apply_head(*c.head, raw, 2, out), std::runtime_error);
}

TEST(ApplyHead, RejectsLongRawBuffer)
{
    using visionpilot::models::apply_head;
    const auto c = MergedContract::from_json_string(kV7Contract);
    const float raw[4] = {0.f, 0.f, 0.f, 0.f};
    visionpilot::models::AutoDriveOutput out;
    EXPECT_THROW(apply_head(*c.head, raw, 4, out), std::runtime_error);
}

TEST(ApplyHead, RejectsUnknownHeadOutputName)
{
    using visionpilot::models::apply_head;
    using visionpilot::models::ContractHead;

    // Built by hand rather than parsed: parse_head() now refuses an unknown
    // destination at startup, so this row can no longer come through the
    // parser. apply_head()'s own check survives as the defensive fallback for
    // a hand-built ContractHead, and this test keeps it honest.
    ContractHead head;
    head.output = "drive_head_raw";
    head.alpha  = {1.0f};
    head.map    = {{"drive_mystery", "relu"}};

    const float raw[1] = {1.0f};
    visionpilot::models::AutoDriveOutput out;
    try {
        apply_head(head, raw, 1, out);
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find("unknown head output 'drive_mystery'"),
                  std::string::npos) << msg;
    }
}

TEST(MergedContract, RejectsUnknownHeadMapOutputAtParseTime)
{
    // A one-character typo in contract.json must be one startup refusal
    // naming the bad row, not an apply_head() throw at frame rate.
    try {
        MergedContract::from_json_string(R"({
          "head": {"output": "drive_head_raw", "alpha": [1.0],
                   "map": [["drive_mystery", "relu"]]}
        })");
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find("head.map names unknown output 'drive_mystery'"),
                  std::string::npos) << msg;
        // The activation is valid; only the destination is wrong, so the
        // activation guard must not be the one that fired.
        EXPECT_EQ(msg.find("unknown activation"), std::string::npos) << msg;
    }
}

TEST(MergedContract, AcceptsEveryValidHeadMapDestination)
{
    // The guard above must refuse none of the three real destinations, in
    // any order.
    const auto c = MergedContract::from_json_string(R"({
      "head": {"output": "drive_head_raw", "alpha": [1.0, 2.0, 3.0],
               "map": [["drive_flag_logit", "none"],
                       ["drive_curvature", "tanh"],
                       ["drive_distance", "relu"]]}
    })");
    ASSERT_TRUE(c.head.has_value());
    EXPECT_EQ(c.head->map.size(), 3u);
}

TEST(ApplySteerXp, UniformLogitsGiveMeanPosition)
{
    using visionpilot::models::apply_steer_xp;
    const auto c = MergedContract::from_json_string(kV7Contract);
    ASSERT_TRUE(c.steer_xp.has_value());

    // All-equal logits -> uniform softmax -> expectation is the mean index.
    // mean(arange(256)) = 127.5, divided by 256 -> 0.498046875.
    std::vector<float> logits(64 * 256, 3.14f);

    visionpilot::models::AutoSteerOutput out;
    apply_steer_xp(*c.steer_xp, logits.data(), logits.size(), out);

    for (size_t r = 0; r < out.xp.size(); ++r) {
        EXPECT_NEAR(out.xp[r], 127.5f / 256.0f, 1e-6f) << "row " << r;
    }
}

TEST(ApplySteerXp, OneHotLogitsGiveThatPosition)
{
    using visionpilot::models::apply_steer_xp;
    const auto c = MergedContract::from_json_string(kV7Contract);

    // Row r peaks hard at column r*4, so the expectation lands there.
    std::vector<float> logits(64 * 256, -60.0f);
    for (int r = 0; r < 64; ++r) logits[r * 256 + r * 4] = 60.0f;

    visionpilot::models::AutoSteerOutput out;
    apply_steer_xp(*c.steer_xp, logits.data(), logits.size(), out);

    for (int r = 0; r < 64; ++r) {
        EXPECT_NEAR(out.xp[r], (r * 4) / 256.0f, 1e-5f) << "row " << r;
    }
}

TEST(ApplySteerXp, IsStableAgainstLargeLogits)
{
    using visionpilot::models::apply_steer_xp;
    const auto c = MergedContract::from_json_string(kV7Contract);

    // Without max-subtraction these would overflow expf and produce NaN.
    std::vector<float> logits(64 * 256, 10000.0f);

    visionpilot::models::AutoSteerOutput out;
    apply_steer_xp(*c.steer_xp, logits.data(), logits.size(), out);

    for (size_t r = 0; r < out.xp.size(); ++r) {
        EXPECT_FALSE(std::isnan(out.xp[r])) << "row " << r;
        EXPECT_NEAR(out.xp[r], 127.5f / 256.0f, 1e-6f);
    }
}

TEST(ApplySteerXp, SetsValidAndMarksSteerUsable)
{
    using visionpilot::models::apply_steer_xp;
    const auto c = MergedContract::from_json_string(kV7Contract);
    std::vector<float> logits(64 * 256, 0.0f);

    visionpilot::models::AutoSteerOutput out;
    ASSERT_FALSE(out.valid);
    apply_steer_xp(*c.steer_xp, logits.data(), logits.size(), out);
    EXPECT_TRUE(out.valid);
}

TEST(ApplySteerXp, RejectsCountThatIsNotAMultipleOfPositions)
{
    using visionpilot::models::apply_steer_xp;
    const auto c = MergedContract::from_json_string(kV7Contract);
    std::vector<float> logits(64 * 256 - 1, 0.0f);

    visionpilot::models::AutoSteerOutput out;
    EXPECT_THROW(apply_steer_xp(*c.steer_xp, logits.data(), logits.size(), out),
                 std::runtime_error);
}

TEST(ApplySteerXp, RejectsWrongRowCount)
{
    using visionpilot::models::apply_steer_xp;
    const auto c = MergedContract::from_json_string(kV7Contract);
    // 32 rows, not the 64 AutoSteerOutput::xp holds.
    std::vector<float> logits(32 * 256, 0.0f);

    visionpilot::models::AutoSteerOutput out;
    EXPECT_THROW(apply_steer_xp(*c.steer_xp, logits.data(), logits.size(), out),
                 std::runtime_error);
}

TEST(ApplySteerXp, RejectsTooManyRows)
{
    using visionpilot::models::apply_steer_xp;
    const auto c = MergedContract::from_json_string(kV7Contract);
    // 65 rows: a valid multiple of positions, but more than the 64
    // AutoSteerOutput::xp holds.
    std::vector<float> logits(65 * 256, 0.0f);

    visionpilot::models::AutoSteerOutput out;
    EXPECT_THROW(apply_steer_xp(*c.steer_xp, logits.data(), logits.size(), out),
                 std::runtime_error);
}

TEST(ApplySteerXp, RejectsNonPositivePositions)
{
    using visionpilot::models::apply_steer_xp;
    using visionpilot::models::ContractSteerXp;

    ContractSteerXp rule;
    rule.logits    = "steer_silu_41";
    rule.positions = 0;
    rule.div       = 256.0f;

    std::vector<float> logits(64 * 256, 0.0f);
    visionpilot::models::AutoSteerOutput out;
    EXPECT_THROW(apply_steer_xp(rule, logits.data(), logits.size(), out),
                 std::runtime_error);
}

TEST(ApplySteerXp, RejectsZeroDiv)
{
    using visionpilot::models::apply_steer_xp;
    using visionpilot::models::ContractSteerXp;

    ContractSteerXp rule;
    rule.logits    = "steer_silu_41";
    rule.positions = 256;
    rule.div       = 0.0f;

    std::vector<float> logits(64 * 256, 0.0f);
    visionpilot::models::AutoSteerOutput out;
    EXPECT_THROW(apply_steer_xp(rule, logits.data(), logits.size(), out),
                 std::runtime_error);
}

TEST(AssembleSpeed, ConcatenatesLevelsAndAppliesStride)
{
    using visionpilot::models::assemble_speed;
    using visionpilot::models::SpeedLevelTensors;

    // Two tiny levels: 1x2 at stride 8, and 1x1 at stride 16. K = 2 classes.
    // Level A boxes [1,4,1,2] in grid units, channel-major.
    const float box_a[8] = {1, 2,    // cx
                            3, 4,    // cy
                            5, 6,    // w
                            7, 8};   // h
    const float cls_a[4] = {0.1f, 0.2f,   // class 0
                            0.3f, 0.4f};  // class 1
    // Level B boxes [1,4,1,1].
    const float box_b[4] = {9, 10, 11, 12};
    const float cls_b[2] = {0.5f, 0.6f};

    std::vector<SpeedLevelTensors> levels = {
        {box_a, cls_a, 1, 2, 8,  2, 8, 4},
        {box_b, cls_b, 1, 1, 16, 2, 4, 2},
    };

    const auto a = assemble_speed(levels);

    EXPECT_EQ(a.channels, 6);   // 4 + K
    EXPECT_EQ(a.anchors, 3);    // 2 + 1
    ASSERT_EQ(a.data.size(), 18u);

    const int64_t N = a.anchors;
    // cx row: level A scaled by 8, then level B scaled by 16.
    EXPECT_FLOAT_EQ(a.data[0 * N + 0], 1 * 8);
    EXPECT_FLOAT_EQ(a.data[0 * N + 1], 2 * 8);
    EXPECT_FLOAT_EQ(a.data[0 * N + 2], 9 * 16);
    // h row.
    EXPECT_FLOAT_EQ(a.data[3 * N + 0], 7 * 8);
    EXPECT_FLOAT_EQ(a.data[3 * N + 2], 12 * 16);
    // Class rows are copied without scaling.
    EXPECT_FLOAT_EQ(a.data[4 * N + 0], 0.1f);
    EXPECT_FLOAT_EQ(a.data[4 * N + 2], 0.5f);
    EXPECT_FLOAT_EQ(a.data[5 * N + 1], 0.4f);
    EXPECT_FLOAT_EQ(a.data[5 * N + 2], 0.6f);
}

TEST(AssembleSpeed, ProducesV7ShapeWithFourClasses)
{
    using visionpilot::models::assemble_speed;
    using visionpilot::models::SpeedLevelTensors;

    // Zero-filled tensors sized like the real v7 levels; only shapes matter.
    // K = 4, matching cls.reshape(1, 4, -1) in both reference implementations.
    std::vector<float> b0(4 * 64 * 128), c0(4 * 64 * 128);
    std::vector<float> b1(4 * 32 * 64),  c1(4 * 32 * 64);
    std::vector<float> b2(4 * 16 * 32),  c2(4 * 16 * 32);

    std::vector<SpeedLevelTensors> levels = {
        {b0.data(), c0.data(), 64, 128, 8,  4, b0.size(), c0.size()},
        {b1.data(), c1.data(), 32, 64,  16, 4, b1.size(), c1.size()},
        {b2.data(), c2.data(), 16, 32,  32, 4, b2.size(), c2.size()},
    };

    const auto a = assemble_speed(levels);
    EXPECT_EQ(a.anchors, 10752);
    EXPECT_EQ(a.channels, 8);
    EXPECT_EQ(a.data.size(), 8u * 10752u);
}

TEST(AssembleSpeed, RejectsInconsistentClassCount)
{
    using visionpilot::models::assemble_speed;
    using visionpilot::models::SpeedLevelTensors;

    const float box[4] = {0, 0, 0, 0};
    const float cls[2] = {0, 0};
    std::vector<SpeedLevelTensors> levels = {
        {box, cls, 1, 1, 8,  2, 4, 2},
        {box, cls, 1, 1, 16, 1, 4, 2},
    };
    EXPECT_THROW(assemble_speed(levels), std::runtime_error);
}

TEST(AssembleSpeed, RejectsEmptyLevelList)
{
    using visionpilot::models::assemble_speed;
    using visionpilot::models::SpeedLevelTensors;
    EXPECT_THROW(assemble_speed(std::vector<SpeedLevelTensors>{}),
                 std::runtime_error);
}

TEST(AssembleSpeed, RejectsMismatchedBoxCount)
{
    using visionpilot::models::assemble_speed;
    using visionpilot::models::SpeedLevelTensors;

    // h=1, w=2 requires box_count == 4*1*2 == 8; report 7 instead, as if
    // the contract's declared geometry disagreed with the real tensor.
    const float box[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    const float cls[2] = {0, 0};
    std::vector<SpeedLevelTensors> levels = {
        {box, cls, 1, 2, 8, 1, /*box_count=*/7, /*cls_count=*/2},
    };
    EXPECT_THROW(assemble_speed(levels), std::runtime_error);
}

TEST(AssembleSpeed, RejectsMismatchedClsCount)
{
    using visionpilot::models::assemble_speed;
    using visionpilot::models::SpeedLevelTensors;

    // h=1, w=2, num_classes=1 requires cls_count == 1*1*2 == 2; report 3
    // instead, as if the contract's declared geometry disagreed with the
    // real tensor.
    const float box[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    const float cls[3] = {0, 0, 0};
    std::vector<SpeedLevelTensors> levels = {
        {box, cls, 1, 2, 8, 1, /*box_count=*/8, /*cls_count=*/3},
    };
    EXPECT_THROW(assemble_speed(levels), std::runtime_error);
}
