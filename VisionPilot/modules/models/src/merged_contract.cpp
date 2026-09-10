#include "models/merged_contract.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <vector>

namespace visionpilot::models {

namespace {

ContractHead parse_head(const nlohmann::json& j)
{
    ContractHead h;
    h.output = j.at("output").get<std::string>();
    h.alpha  = j.at("alpha").get<std::vector<float>>();

    for (const auto& row : j.at("map")) {
        if (!row.is_array() || row.size() != 2) {
            throw std::runtime_error(
                "[MergedContract] head.map rows must be [output, activation]");
        }
        ContractHeadMap m;
        m.output     = row[0].get<std::string>();
        m.activation = row[1].get<std::string>();
        if (m.activation != "relu" && m.activation != "tanh" &&
            m.activation != "none") {
            throw std::runtime_error(
                "[MergedContract] unknown activation '" + m.activation +
                "' for head output '" + m.output + "'");
        }
        // The destination is validated here, beside the activation, so a typo
        // in contract.json is one startup refusal naming the bad row rather
        // than an apply_head() throw on every single frame.
        if (m.output != "drive_distance" && m.output != "drive_curvature" &&
            m.output != "drive_flag_logit") {
            throw std::runtime_error(
                "[MergedContract] head.map names unknown output '" + m.output +
                "'. Expected drive_distance, drive_curvature, or "
                "drive_flag_logit");
        }
        h.map.push_back(std::move(m));
    }

    if (h.alpha.size() != h.map.size()) {
        throw std::runtime_error(
            "[MergedContract] head.alpha has " + std::to_string(h.alpha.size()) +
            " entries but head.map has " + std::to_string(h.map.size()));
    }
    if (h.alpha.empty()) {
        throw std::runtime_error("[MergedContract] head.alpha is empty");
    }
    for (size_t i = 0; i < h.alpha.size(); ++i) {
        if (h.alpha[i] == 0.f) {
            throw std::runtime_error(
                "[MergedContract] head.alpha[" + std::to_string(i) + "] is zero");
        }
    }
    return h;
}

ContractSteerXp parse_steer_xp(const nlohmann::json& j)
{
    ContractSteerXp s;
    s.logits    = j.at("logits").get<std::string>();
    s.positions = j.at("positions").get<int>();
    s.div       = j.at("div").get<float>();

    if (s.positions <= 0) {
        throw std::runtime_error(
            "[MergedContract] steer_xp.positions must be positive, got " +
            std::to_string(s.positions));
    }
    if (s.div == 0.f) {
        throw std::runtime_error("[MergedContract] steer_xp.div is zero");
    }
    return s;
}

ContractSpeed parse_speed(const nlohmann::json& j)
{
    ContractSpeed s;
    for (const auto& lv : j.at("levels")) {
        ContractSpeedLevel l;
        l.box    = lv.at("box").get<std::string>();
        l.cls    = lv.at("cls").get<std::string>();
        l.stride = lv.at("stride").get<int>();

        const auto hw = lv.at("hw");
        if (!hw.is_array() || hw.size() != 2) {
            throw std::runtime_error("[MergedContract] speed level hw must be [h, w]");
        }
        l.h = hw[0].get<int>();
        l.w = hw[1].get<int>();

        if (l.stride <= 0 || l.h <= 0 || l.w <= 0) {
            throw std::runtime_error(
                "[MergedContract] speed level '" + l.box +
                "' has non-positive stride or extent");
        }
        s.levels.push_back(std::move(l));
    }
    if (s.levels.empty()) {
        throw std::runtime_error("[MergedContract] speed.levels is empty");
    }
    return s;
}

}  // namespace

namespace {

float apply_activation(const std::string& kind, float v)
{
    if (kind == "relu") return v > 0.f ? v : 0.f;
    if (kind == "tanh") return std::tanh(v);
    return v;  // "none" — validated at parse time
}

}  // namespace

void apply_head(const ContractHead& head, const float* raw, size_t raw_count,
                AutoDriveOutput& out)
{
    if (raw_count != head.map.size()) {
        throw std::runtime_error(
            "[MergedContract] head tensor has " + std::to_string(raw_count) +
            " elements but the contract's head.map describes " +
            std::to_string(head.map.size()) + " rows");
    }

    for (size_t i = 0; i < head.map.size(); ++i) {
        const auto& m = head.map[i];
        const float v = apply_activation(m.activation, raw[i] / head.alpha[i]);

        if (m.output == "drive_distance") {
            out.dist_normalized = v;
        } else if (m.output == "drive_curvature") {
            out.curvature_raw = v;
        } else if (m.output == "drive_flag_logit") {
            out.flag_prob = 1.f / (1.f + std::exp(-v));
        } else {
            // Unreachable through the parser: parse_head() refuses any other
            // destination name at startup. Kept as a defensive fallback for a
            // ContractHead built by hand rather than parsed.
            throw std::runtime_error(
                "[MergedContract] unknown head output '" + m.output +
                "'. Expected drive_distance, drive_curvature, or "
                "drive_flag_logit");
        }
    }
    out.valid = true;
}

void apply_steer_xp(const ContractSteerXp& rule, const float* logits,
                    size_t count, AutoSteerOutput& out)
{
    if (rule.positions <= 0) {
        throw std::runtime_error(
            "[MergedContract] steer_xp.positions must be positive, got " +
            std::to_string(rule.positions));
    }
    if (rule.div == 0.f) {
        throw std::runtime_error("[MergedContract] steer_xp.div is zero");
    }

    const size_t positions = static_cast<size_t>(rule.positions);

    if (count == 0 || count % positions != 0) {
        throw std::runtime_error(
            "[MergedContract] steer_xp logits hold " + std::to_string(count) +
            " elements, which is not a positive multiple of positions=" +
            std::to_string(positions));
    }

    const size_t rows = count / positions;
    if (rows != out.xp.size()) {
        throw std::runtime_error(
            "[MergedContract] steer_xp logits describe " +
            std::to_string(rows) + " rows but AutoSteerOutput::xp holds " +
            std::to_string(out.xp.size()));
    }

    for (size_t r = 0; r < rows; ++r) {
        const float* row = logits + r * positions;

        // Subtract the row max before exponentiating, as the reference does;
        // the raw logits are unbounded and expf would overflow.
        double max_v = row[0];
        for (size_t i = 1; i < positions; ++i) {
            if (static_cast<double>(row[i]) > max_v) max_v = row[i];
        }

        double sum = 0.0;
        double acc = 0.0;
        for (size_t i = 0; i < positions; ++i) {
            const double e = std::exp(static_cast<double>(row[i]) - max_v);
            sum += e;
            acc += e * static_cast<double>(i);
        }

        // sum >= 1 because the max element contributes exp(0) = 1.
        out.xp[r] = static_cast<float>((acc / sum) / rule.div);
    }
    out.valid = true;
}

AssembledSpeed assemble_speed(const std::vector<SpeedLevelTensors>& levels)
{
    if (levels.empty()) {
        throw std::runtime_error("[MergedContract] no speed levels to assemble");
    }

    const int num_classes = levels.front().num_classes;
    if (num_classes <= 0) {
        throw std::runtime_error(
            "[MergedContract] speed level has non-positive class count");
    }

    int64_t total = 0;
    for (const auto& l : levels) {
        if (l.box == nullptr || l.cls == nullptr) {
            throw std::runtime_error(
                "[MergedContract] speed level has a null tensor");
        }
        if (l.num_classes != num_classes) {
            throw std::runtime_error(
                "[MergedContract] speed levels disagree on class count: " +
                std::to_string(num_classes) + " vs " +
                std::to_string(l.num_classes));
        }
        if (l.h <= 0 || l.w <= 0 || l.stride <= 0) {
            throw std::runtime_error(
                "[MergedContract] speed level has non-positive extent or stride");
        }

        // Guard the buffers against a contract that mis-describes the
        // graph's real outputs, before either is dereferenced below.
        const int64_t expected_box =
            4 * static_cast<int64_t>(l.h) * static_cast<int64_t>(l.w);
        if (static_cast<int64_t>(l.box_count) != expected_box) {
            throw std::runtime_error(
                "[MergedContract] speed level box tensor has " +
                std::to_string(l.box_count) + " elements but 4*h*w requires " +
                std::to_string(expected_box));
        }
        const int64_t expected_cls = static_cast<int64_t>(num_classes) *
                                      static_cast<int64_t>(l.h) *
                                      static_cast<int64_t>(l.w);
        if (static_cast<int64_t>(l.cls_count) != expected_cls) {
            throw std::runtime_error(
                "[MergedContract] speed level cls tensor has " +
                std::to_string(l.cls_count) + " elements but num_classes*h*w "
                "requires " + std::to_string(expected_cls));
        }

        total += static_cast<int64_t>(l.h) * l.w;
    }

    AssembledSpeed out;
    out.channels = 4 + num_classes;
    out.anchors  = total;
    out.data.assign(static_cast<size_t>(out.channels * out.anchors), 0.f);

    int64_t offset = 0;
    for (const auto& l : levels) {
        const int64_t n_l = static_cast<int64_t>(l.h) * l.w;
        const float   s   = static_cast<float>(l.stride);

        for (int64_t c = 0; c < 4; ++c) {
            const float* src = l.box + c * n_l;
            float*       dst = out.data.data() + c * out.anchors + offset;
            for (int64_t i = 0; i < n_l; ++i) dst[i] = src[i] * s;
        }
        for (int64_t k = 0; k < num_classes; ++k) {
            const float* src = l.cls + k * n_l;
            float*       dst = out.data.data() + (4 + k) * out.anchors + offset;
            for (int64_t i = 0; i < n_l; ++i) dst[i] = src[i];
        }
        offset += n_l;
    }
    return out;
}

MergedContract MergedContract::from_json_string(const std::string& text)
{
    nlohmann::json j;
    try {
        j = nlohmann::json::parse(text);
    } catch (const nlohmann::json::exception& e) {
        throw std::runtime_error(
            std::string("[MergedContract] malformed JSON: ") + e.what());
    }

    MergedContract c;
    try {
        if (j.contains("attn_mode")) {
            c.attn_mode = j.at("attn_mode").get<std::string>();
        }
        if (j.contains("passthrough")) {
            c.passthrough = j.at("passthrough").get<std::vector<std::string>>();
        }
        if (j.contains("head")) {
            c.head = parse_head(j.at("head"));
        }
        if (j.contains("steer_xp")) {
            c.steer_xp = parse_steer_xp(j.at("steer_xp"));
        }
        if (j.contains("speed")) {
            c.speed = parse_speed(j.at("speed"));
        }
    } catch (const nlohmann::json::exception& e) {
        throw std::runtime_error(
            std::string("[MergedContract] invalid contract: ") + e.what());
    }
    return c;
}

MergedContract MergedContract::from_file(const std::string& path)
{
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("[MergedContract] cannot open " + path);
    }
    const std::string text((std::istreambuf_iterator<char>(in)),
                            std::istreambuf_iterator<char>());
    return from_json_string(text);
}

std::string resolve_contract_path(const std::string& model_path,
                                  const std::string& artifacts_dir)
{
    namespace fs = std::filesystem;

    if (!model_path.empty()) {
        const std::string sidecar = model_path + ".contract.json";
        if (fs::exists(sidecar)) return sidecar;
    }
    if (artifacts_dir.empty()) return {};

    const auto in_dir = fs::path(artifacts_dir) / "contract.json";
    if (fs::exists(in_dir)) return in_dir.string();

    // A contract is distributed under the sidecar name of the model it was
    // rewritten from ("v7_frozen.onnx.contract.json"), and under the renesas
    // provider there is nothing to hang that name off: resolve_merged_target()
    // has no model path to pass, and the file the session actually loads is
    // the compiled qdq_inserted_* one, not the model the sidecar is named
    // after. So a sidecar lookup keyed on the resolved model would not help
    // either. Accept the distributed name where it lands instead of requiring
    // every user to rename the file.
    std::error_code ec;
    if (!fs::is_directory(artifacts_dir, ec) || ec) return {};

    static constexpr std::string_view kSuffix = ".contract.json";
    std::vector<std::string> matches;
    for (const auto& e : fs::directory_iterator(artifacts_dir)) {
        if (!e.is_regular_file()) continue;
        const std::string name = e.path().filename().string();
        if (name.size() > kSuffix.size() &&
            name.compare(name.size() - kSuffix.size(), kSuffix.size(),
                         kSuffix) == 0) {
            matches.push_back(e.path().string());
        }
    }

    if (matches.empty()) return {};
    if (matches.size() > 1) {
        // Picking one would silently decide which rewrite's host
        // postprocessing runs, and a wrong pick is not detectable downstream:
        // the head alpha, the lane rule and the speed geometry would all be
        // read from a contract that does not describe this graph.
        std::sort(matches.begin(), matches.end());
        std::string msg =
            "[MergedContract] " + artifacts_dir +
            " holds more than one *.contract.json, so which contract applies "
            "is ambiguous. Keep exactly one, rename the intended one to "
            "contract.json, or set model.contract explicitly. Found:";
        for (const auto& m : matches) msg += "\n  " + m;
        throw std::runtime_error(msg);
    }
    return matches.front();
}

bool is_drive_head_raw_output(const std::string& name)
{
    return name == "drive_head_raw";
}

bool is_speed_level_box_output(const std::string& name)
{
    // speed_l<N>_box exists only after the R7 per-level DFL rewrite.
    return name.rfind("speed_l", 0) == 0 &&
           name.size() > 4 && name.compare(name.size() - 4, 4, "_box") == 0;
}

bool has_rewrite_signature_output(const std::vector<std::string>& output_names)
{
    for (const auto& n : output_names) {
        if (is_drive_head_raw_output(n) || is_speed_level_box_output(n)) {
            return true;
        }
    }
    return false;
}

}  // namespace visionpilot::models
