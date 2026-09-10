#include <gtest/gtest.h>
#include <engine/onnx_engine.hpp>
#include <models/merged_backend.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;
namespace models = visionpilot::models;
using visionpilot::engine::Config;
using visionpilot::engine::OnnxEngine;
using visionpilot::engine::parse_profile_providers;
using visionpilot::engine::resolve_renesas_artifacts;

namespace {

void touch(const fs::path& p)
{
    fs::create_directories(p.parent_path());
    std::ofstream(p) << "x";
}

// A directory unique to this process/run: two concurrent test runs (or two
// users on a shared host) must never collide or clobber each other's
// fixtures. Mirrors ScopedTempDir in test_merged_contract.cpp.
fs::path make_unique_temp_dir(const std::string& tag)
{
    const auto stamp =
        std::chrono::steady_clock::now().time_since_epoch().count();
    return fs::temp_directory_path() / ("vp_" + tag + "_" + std::to_string(stamp));
}

// True iff resolve_renesas_artifacts's error message lists `path` as its own
// "Missing:" entry (i.e. the entry's exact text, not merely a substring of
// it or of a longer entry). A plain substring search would misfire two
// ways: "legalized_*.onnx" contains "nnx", and the nnx/ entry is itself a
// path-prefix of the nnx/manifest.json entry.
bool has_missing_entry(const std::string& msg, const std::string& path)
{
    const std::string needle = "\n  " + path;
    for (auto pos = msg.find(needle); pos != std::string::npos;
         pos = msg.find(needle, pos + 1)) {
        const auto after = pos + needle.size();
        if (after == msg.size() || msg[after] == '\n') return true;
    }
    return false;
}

class ArtifactsDir : public ::testing::Test {
protected:
    void SetUp() override
    {
        root_ = make_unique_temp_dir("artifacts_test");
        dir_ = root_ / "artifacts_set_a";
        fs::create_directories(dir_);
    }
    void TearDown() override { fs::remove_all(root_); }

    void add_nnx_with_manifest()
    {
        fs::create_directories(dir_ / "nnx");
        touch(dir_ / "nnx" / "manifest.json");
    }
    void add_fused_subgraphs() { fs::create_directories(dir_ / "fused_subgraphs"); }
    void add_legalized() { touch(dir_ / "legalized_model.onnx"); }

    void complete_set()
    {
        add_nnx_with_manifest();
        add_fused_subgraphs();
        add_legalized();
    }

    fs::path root_, dir_;
};

}  // namespace

TEST_F(ArtifactsDir, ResolvesCompleteSet)
{
    complete_set();
    const auto a = resolve_renesas_artifacts(dir_.string());

    EXPECT_EQ(a.model, (dir_ / "legalized_model.onnx").string());
    EXPECT_EQ(a.manifest, (dir_ / "nnx" / "manifest.json").string());
    EXPECT_FALSE(a.qdq_inserted);
    // base_path is the PARENT of the artifacts directory.
    EXPECT_EQ(a.base, root_.string());
}

TEST_F(ArtifactsDir, TrailingSeparatorStillYieldsTheParentAsBase)
{
    // A configured path with a trailing separator leaves an empty filename,
    // so a bare parent_path() would return the artifacts directory itself.
    // The execution provider would then resolve the artifacts' recorded
    // absolute paths against the wrong root and fail with "nnx load failed:"
    // naming a path that exists only on the compile host.
    complete_set();
    const auto a = resolve_renesas_artifacts(dir_.string() + "/");

    EXPECT_EQ(a.base, root_.string());
}

TEST_F(ArtifactsDir, RelativeSingleComponentPathStillYieldsAnAbsoluteBase)
{
    // parent_path() of a relative one-component path is empty, which would
    // hand the execution provider an empty base_path.
    complete_set();

    const auto cwd = fs::current_path();
    fs::current_path(root_);
    const auto a = resolve_renesas_artifacts("artifacts_set_a");
    fs::current_path(cwd);

    EXPECT_FALSE(a.base.empty());
    EXPECT_EQ(fs::path(a.base).filename(), root_.filename());
}

TEST_F(ArtifactsDir, PrefersQdqInsertedModel)
{
    complete_set();
    touch(dir_ / "qdq_inserted_legalized_model.onnx");

    const auto a = resolve_renesas_artifacts(dir_.string());
    EXPECT_EQ(a.model, (dir_ / "qdq_inserted_legalized_model.onnx").string());
    EXPECT_TRUE(a.qdq_inserted);
}

TEST_F(ArtifactsDir, PicksLexicographicallyFirstOnMultipleMatches)
{
    add_nnx_with_manifest();
    add_fused_subgraphs();
    touch(dir_ / "legalized_b.onnx");
    touch(dir_ / "legalized_a.onnx");

    // directory_iterator's enumeration order is unspecified; the resolver
    // must sort so the pick does not depend on it.
    const auto a = resolve_renesas_artifacts(dir_.string());
    EXPECT_EQ(a.model, (dir_ / "legalized_a.onnx").string());
}

TEST_F(ArtifactsDir, RejectsMissingNnxDir)
{
    add_fused_subgraphs();
    add_legalized();

    try {
        resolve_renesas_artifacts(dir_.string());
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_TRUE(has_missing_entry(msg, (dir_ / "nnx").string()));
        EXPECT_FALSE(has_missing_entry(msg, (dir_ / "fused_subgraphs").string()));
        EXPECT_FALSE(has_missing_entry(msg, (dir_ / "legalized_*.onnx").string()));
    }
}

TEST_F(ArtifactsDir, RejectsMissingManifestJson)
{
    fs::create_directories(dir_ / "nnx");  // nnx/ exists, manifest.json does not
    add_fused_subgraphs();
    add_legalized();

    try {
        resolve_renesas_artifacts(dir_.string());
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_TRUE(has_missing_entry(msg, (dir_ / "nnx" / "manifest.json").string()));
        // The nnx/ entry itself must NOT appear: nnx/ exists, only the file
        // inside it is missing, and that distinction must be visible in the
        // message (not collapsed into "nnx/ is missing").
        EXPECT_FALSE(has_missing_entry(msg, (dir_ / "nnx").string()));
        EXPECT_FALSE(has_missing_entry(msg, (dir_ / "fused_subgraphs").string()));
    }
}

TEST_F(ArtifactsDir, RejectsMissingFusedSubgraphsDir)
{
    add_nnx_with_manifest();
    add_legalized();

    try {
        resolve_renesas_artifacts(dir_.string());
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_TRUE(has_missing_entry(msg, (dir_ / "fused_subgraphs").string()));
        EXPECT_FALSE(has_missing_entry(msg, (dir_ / "nnx").string()));
        EXPECT_FALSE(has_missing_entry(msg, (dir_ / "nnx" / "manifest.json").string()));
    }
}

TEST_F(ArtifactsDir, RejectsMissingLegalizedOnnx)
{
    add_nnx_with_manifest();
    add_fused_subgraphs();

    try {
        resolve_renesas_artifacts(dir_.string());
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_TRUE(has_missing_entry(msg, (dir_ / "legalized_*.onnx").string()));
        EXPECT_FALSE(has_missing_entry(msg, (dir_ / "fused_subgraphs").string()));
        EXPECT_FALSE(has_missing_entry(msg, (dir_ / "nnx").string()));
    }
}

TEST_F(ArtifactsDir, RejectsNonExistentDirectory)
{
    EXPECT_THROW(resolve_renesas_artifacts((root_ / "nope").string()),
                 std::runtime_error);
}

TEST_F(ArtifactsDir, RejectsEmptyArtifactsDir)
{
    EXPECT_THROW(resolve_renesas_artifacts(""), std::runtime_error);
}

TEST_F(ArtifactsDir, RejectsArtifactsDirThatIsARegularFile)
{
    const auto file = root_ / "not_a_directory";
    touch(file);
    EXPECT_THROW(resolve_renesas_artifacts(file.string()), std::runtime_error);
}

// ─── NPU offload profile parsing ─────────────────────────────────────────────

TEST_F(ArtifactsDir, CountsProvidersFromProfileJson)
{
    const auto p = root_ / "profile.json";
    fs::create_directories(root_);
    std::ofstream(p) << R"([
      {"cat":"Session","name":"model_run","dur":1000},
      {"cat":"Node","name":"Conv_1_kernel_time","dur":10,
       "args":{"provider":"RenesasExecutionProvider"}},
      {"cat":"Node","name":"Conv_2_kernel_time","dur":12,
       "args":{"provider":"RenesasExecutionProvider"}},
      {"cat":"Node","name":"Tanh_1_kernel_time","dur":1,
       "args":{"provider":"CPUExecutionProvider"}},
      {"cat":"Node","name":"Conv_1_fence_before","dur":0,
       "args":{"provider":"RenesasExecutionProvider"}}
    ])";

    const auto hist = parse_profile_providers(p.string());
    EXPECT_EQ(hist.at("RenesasExecutionProvider"), 2);
    EXPECT_EQ(hist.at("CPUExecutionProvider"), 1);
    // Non-kernel_time events must not be counted.
    EXPECT_EQ(hist.size(), 2u);
}

TEST_F(ArtifactsDir, ProfileWithNoNodeEventsIsEmpty)
{
    const auto p = root_ / "empty_profile.json";
    fs::create_directories(root_);
    std::ofstream(p) << R"([{"cat":"Session","name":"model_run","dur":1}])";
    EXPECT_TRUE(parse_profile_providers(p.string()).empty());
}

TEST_F(ArtifactsDir, MissingProfileThrows)
{
    const auto p = (root_ / "nope.json").string();

    try {
        parse_profile_providers(p);
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        // The path is what separates an environmental fault (wrong path,
        // unwritable CWD) from a genuine offload failure -- it must be
        // named, not just "something went wrong."
        EXPECT_NE(msg.find(p), std::string::npos) << msg;
        // This is a file-access problem, not an offload verdict: it must
        // not read like one of the gate's own pass/fail messages.
        EXPECT_EQ(msg.find("offload"), std::string::npos) << msg;
        EXPECT_EQ(msg.find("NPU"), std::string::npos) << msg;
    }
}

TEST_F(ArtifactsDir, EmptyProfilePathThrows)
{
    try {
        parse_profile_providers("");
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        // Must read as "no path was given," not as "cannot open profile "
        // trailing off into nothing.
        EXPECT_NE(msg.find("empty"), std::string::npos) << msg;
    }
}

TEST_F(ArtifactsDir, MalformedProfileJsonThrows)
{
    const auto p = root_ / "malformed_profile.json";
    fs::create_directories(root_);
    std::ofstream(p) << R"({"cat": "Node", "name": )";  // truncated, invalid JSON

    try {
        parse_profile_providers(p.string());
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find(p.string()), std::string::npos) << msg;
    }
}

TEST_F(ArtifactsDir, ZeroByteProfileFileThrows)
{
    const auto p = root_ / "zero_byte_profile.json";
    fs::create_directories(root_);
    std::ofstream out(p);  // create, write nothing
    out.close();

    try {
        parse_profile_providers(p.string());
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find(p.string()), std::string::npos) << msg;
    }
}

TEST_F(ArtifactsDir, NonArrayProfileJsonThrows)
{
    // Valid JSON, but not the array of events ORT emits. This must be an
    // error, not "no node events found": ORT profiling output is always a
    // top-level array, so anything else means this is not the profile the
    // gate thinks it is (wrong file, wrong tool, truncated write), and
    // reporting an empty histogram for it would let that environmental
    // problem masquerade as "the NPU did no work."
    const auto p = root_ / "object_profile.json";
    fs::create_directories(root_);
    std::ofstream(p) << R"({"not": "an array"})";

    try {
        parse_profile_providers(p.string());
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find(p.string()), std::string::npos) << msg;
    }
}

TEST_F(ArtifactsDir, ArrayWithNonObjectElementIsSkipped)
{
    const auto p = root_ / "non_object_element_profile.json";
    fs::create_directories(root_);
    std::ofstream(p) << R"([
      42,
      "a stray string",
      {"cat":"Node","name":"Conv_1_kernel_time","dur":10,
       "args":{"provider":"RenesasExecutionProvider"}}
    ])";

    const auto hist = parse_profile_providers(p.string());
    EXPECT_EQ(hist.at("RenesasExecutionProvider"), 1);
    EXPECT_EQ(hist.size(), 1u);
}

TEST_F(ArtifactsDir, KernelTimeEventMissingArgsOrProviderIsSkipped)
{
    const auto p = root_ / "missing_fields_profile.json";
    fs::create_directories(root_);
    std::ofstream(p) << R"([
      {"cat":"Node","name":"Conv_1_kernel_time","dur":10},
      {"cat":"Node","name":"Conv_2_kernel_time","dur":10,"args":{}},
      {"cat":"Node","name":"Conv_3_kernel_time","dur":10,
       "args":{"provider":"RenesasExecutionProvider"}}
    ])";

    const auto hist = parse_profile_providers(p.string());
    EXPECT_EQ(hist.at("RenesasExecutionProvider"), 1);
    EXPECT_EQ(hist.size(), 1u);
}

TEST_F(ArtifactsDir, NonStringNameOrProviderIsSkipped)
{
    // A numeric "name" cannot match the "*_kernel_time" suffix check, and a
    // null "provider" cannot be counted as a provider -- both must be
    // skipped, not throw nlohmann::json::type_error escaping uncaught.
    const auto p = root_ / "non_string_fields_profile.json";
    fs::create_directories(root_);
    std::ofstream(p) << R"([
      {"cat":"Node","name":5,"dur":10,
       "args":{"provider":"RenesasExecutionProvider"}},
      {"cat":"Node","name":"Conv_1_kernel_time","dur":10,
       "args":{"provider":null}},
      {"cat":"Node","name":"Conv_2_kernel_time","dur":10,
       "args":{"provider":"RenesasExecutionProvider"}}
    ])";

    const auto hist = parse_profile_providers(p.string());
    EXPECT_EQ(hist.at("RenesasExecutionProvider"), 1);
    EXPECT_EQ(hist.size(), 1u);
}

// ─── The offload gate's decision logic ───────────────────────────────────────
//
// check_offload() takes only a histogram and a required count, so its three
// safety-critical behaviours are testable without a Renesas execution
// provider.

TEST_F(ArtifactsDir, CheckOffloadZeroRequiredMeansAtLeastOne)
{
    // require_npu_nodes's default, 0, must mean "at least one," not
    // "require none."
    EXPECT_THROW(models::check_offload({}, /*required=*/0),
                 std::runtime_error);
    EXPECT_EQ(models::check_offload(
                  {{"RenesasExecutionProvider", 1}}, /*required=*/0),
              1);
}

TEST_F(ArtifactsDir, CheckOffloadThrowsOnZeroRenesasNodes)
{
    const std::map<std::string, int> hist = {{"CPUExecutionProvider", 40}};
    try {
        models::check_offload(hist, /*required=*/0);
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find("0"), std::string::npos) << msg;
        EXPECT_NE(msg.find("RenesasExecutionProvider"), std::string::npos)
            << msg;
    }
}

TEST_F(ArtifactsDir, CheckOffloadThrowsWhenBelowConfiguredFloor)
{
    const std::map<std::string, int> hist = {
        {"RenesasExecutionProvider", 3}, {"CPUExecutionProvider", 1}};

    // Passes its own floor...
    EXPECT_EQ(models::check_offload(hist, /*required=*/3), 3);
    // ...but a recompile that sheds one subgraph to the CPU must fail a
    // higher configured floor, naming both counts.
    try {
        models::check_offload(hist, /*required=*/5);
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find("3"), std::string::npos) << msg;
        EXPECT_NE(msg.find("5"), std::string::npos) << msg;
    }
}

// ─── OnnxEngine-level guards ─────────────────────────────────────────────────
//
// These run before resolve_renesas_artifacts and before
// AppendExecutionProvider, so they are reachable on any host -- no Renesas
// execution provider required.

namespace {

class OnnxEngineRenesasGuards : public ::testing::Test {
protected:
    void SetUp() override { root_ = make_unique_temp_dir("engine_guards_test"); }
    void TearDown() override { fs::remove_all(root_); }

    fs::path root_;
};

}  // namespace

TEST_F(OnnxEngineRenesasGuards, RejectsEmptyArcProgPath)
{
    Config cfg;
    cfg.provider = "renesas";
    cfg.arc_prog_path.clear();
    OnnxEngine engine(cfg);

    try {
        engine.create_session((root_ / "artifacts").string());
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("arc_prog_path"), std::string::npos);
    }
}

TEST_F(OnnxEngineRenesasGuards, RejectsArcProgPathThatIsARegularFile)
{
    const auto file = root_ / "arc_prog_as_a_file";
    touch(file);

    Config cfg;
    cfg.provider = "renesas";
    cfg.arc_prog_path = file.string();
    OnnxEngine engine(cfg);

    try {
        engine.create_session((root_ / "artifacts").string());
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("arc_prog_path"), std::string::npos);
    }
}

TEST_F(OnnxEngineRenesasGuards, RejectsUnknownProvider)
{
    Config cfg;
    cfg.provider = "not_a_real_provider";
    OnnxEngine engine(cfg);

    try {
        engine.create_session((root_ / "model.onnx").string());
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("Unknown provider"), std::string::npos);
    }
}
