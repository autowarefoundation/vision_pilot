#include <gtest/gtest.h>

#include <engine/onnx_engine.hpp>
#include <models/inference.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace vpe = visionpilot::engine;
namespace vpm = visionpilot::models;

namespace {

// A directory unique to this process/run, mirroring ScopedTempDir in
// test_merged_contract.cpp and make_unique_temp_dir in
// test_renesas_artifacts.cpp.
class ScopedTempDir
{
public:
    ScopedTempDir()
        : path_(std::filesystem::temp_directory_path() /
                ("visionpilot_inference_test_" +
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

TEST(ResolveMergedTarget, SplitPathReturnsNulloptForNonRenesas)
{
    vpe::Config engine_cfg;
    engine_cfg.provider = "cuda";
    vpm::Config cfg;  // merged defaults to false

    EXPECT_EQ(vpm::resolve_merged_target(engine_cfg, cfg), std::nullopt);
}

TEST(ResolveMergedTarget, RenesasWithoutMergedIsRefused)
{
    vpe::Config engine_cfg;
    engine_cfg.provider = "renesas";
    vpm::Config cfg;  // merged defaults to false

    try {
        vpm::resolve_merged_target(engine_cfg, cfg);
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find("model.merged = true"), std::string::npos);
    }
}

TEST(ResolveMergedTarget, RenesasWithEmptyArtifactsDirIsRefused)
{
    vpe::Config engine_cfg;
    engine_cfg.provider      = "renesas";
    engine_cfg.artifacts_dir = "";
    vpm::Config cfg;
    cfg.merged = true;

    try {
        vpm::resolve_merged_target(engine_cfg, cfg);
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find("engine.artifacts_dir"), std::string::npos);
    }
}

TEST(ResolveMergedTarget, NonRenesasWithEmptyMergedPathIsRefused)
{
    vpe::Config engine_cfg;
    engine_cfg.provider = "cuda";
    vpm::Config cfg;
    cfg.merged      = true;
    cfg.merged_path = "";

    try {
        vpm::resolve_merged_target(engine_cfg, cfg);
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find("model.merged_path"), std::string::npos);
    }
}

TEST(ResolveMergedTarget, RenesasUsesArtifactsDirIgnoringMergedPath)
{
    vpe::Config engine_cfg;
    engine_cfg.provider      = "renesas";
    engine_cfg.artifacts_dir = "/artifacts/set_a";
    vpm::Config cfg;
    cfg.merged      = true;
    cfg.merged_path = "/should/be/ignored.onnx";
    cfg.contract     = "none";

    const auto target = vpm::resolve_merged_target(engine_cfg, cfg);
    ASSERT_TRUE(target.has_value());
    EXPECT_EQ(target->model_or_dir, "/artifacts/set_a");
}

TEST(ResolveMergedTarget, NonRenesasUsesMergedPath)
{
    vpe::Config engine_cfg;
    engine_cfg.provider      = "cuda";
    engine_cfg.artifacts_dir = "/should/be/ignored";
    vpm::Config cfg;
    cfg.merged      = true;
    cfg.merged_path = "/models/merged.onnx";
    cfg.contract     = "none";

    const auto target = vpm::resolve_merged_target(engine_cfg, cfg);
    ASSERT_TRUE(target.has_value());
    EXPECT_EQ(target->model_or_dir, "/models/merged.onnx");
}

TEST(ResolveMergedTarget, ContractNoneYieldsEmptyPathEvenWhenSidecarExists)
{
    ScopedTempDir dir;
    const auto model_path = (dir.path() / "merged.onnx").string();
    write_file(model_path + ".contract.json", "{}");

    vpe::Config engine_cfg;
    engine_cfg.provider = "cuda";
    vpm::Config cfg;
    cfg.merged      = true;
    cfg.merged_path = model_path;
    cfg.contract     = "none";

    const auto target = vpm::resolve_merged_target(engine_cfg, cfg);
    ASSERT_TRUE(target.has_value());
    EXPECT_EQ(target->contract_path, "");
}

TEST(ResolveMergedTarget, ContractAutoResolvesSidecarForPlainModelPath)
{
    ScopedTempDir dir;
    const auto model_path = (dir.path() / "merged.onnx").string();
    const auto sidecar     = model_path + ".contract.json";
    write_file(sidecar, "{}");

    vpe::Config engine_cfg;
    engine_cfg.provider = "cuda";
    vpm::Config cfg;
    cfg.merged      = true;
    cfg.merged_path = model_path;
    cfg.contract     = "auto";

    const auto target = vpm::resolve_merged_target(engine_cfg, cfg);
    ASSERT_TRUE(target.has_value());
    EXPECT_EQ(target->contract_path, sidecar);
}

TEST(ResolveMergedTarget, ContractAutoResolvesArtifactsDirContractUnderRenesas)
{
    ScopedTempDir dir;
    const auto in_dir = (dir.path() / "contract.json").string();
    write_file(in_dir, "{}");

    vpe::Config engine_cfg;
    engine_cfg.provider      = "renesas";
    engine_cfg.artifacts_dir = dir.path().string();
    vpm::Config cfg;
    cfg.merged  = true;
    cfg.contract = "auto";

    const auto target = vpm::resolve_merged_target(engine_cfg, cfg);
    ASSERT_TRUE(target.has_value());
    EXPECT_EQ(target->contract_path, in_dir);
}

TEST(ResolveMergedTarget, ContractExplicitPathIsPassedThroughVerbatim)
{
    vpe::Config engine_cfg;
    engine_cfg.provider = "cuda";
    vpm::Config cfg;
    cfg.merged      = true;
    cfg.merged_path = "/models/merged.onnx";
    cfg.contract     = "/some/explicit/contract.json";

    const auto target = vpm::resolve_merged_target(engine_cfg, cfg);
    ASSERT_TRUE(target.has_value());
    EXPECT_EQ(target->contract_path, "/some/explicit/contract.json");
}

TEST(ResolveMergedTarget, ContractSentinelsAreCaseInsensitive)
{
    vpe::Config engine_cfg;
    engine_cfg.provider = "cuda";
    vpm::Config cfg;
    cfg.merged      = true;
    cfg.merged_path = "/models/merged.onnx";

    cfg.contract = "None";
    auto none_target = vpm::resolve_merged_target(engine_cfg, cfg);
    ASSERT_TRUE(none_target.has_value());
    EXPECT_EQ(none_target->contract_path, "");

    cfg.contract = "AUTO";
    auto auto_target = vpm::resolve_merged_target(engine_cfg, cfg);
    ASSERT_TRUE(auto_target.has_value());
    // No sidecar exists next to this made-up path, so "Auto" resolves to
    // the same "not found" result "auto" would -- proving it took the
    // sentinel branch rather than being treated as a literal path.
    EXPECT_EQ(auto_target->contract_path, "");
}
