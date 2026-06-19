#include <config/vision_pilot_config.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

std::string trim(const std::string& s)
{
    const auto a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    return s.substr(a, s.find_last_not_of(" \t\r\n") - a + 1);
}

std::string expand_home(std::string p)
{
    if (!p.empty() && p[0] == '~') {
        const char* home = std::getenv("HOME");
        if (!home) throw std::runtime_error("HOME not set, cannot expand ~");
        if (p.size() == 1 || p[1] == '/') p.replace(0, 1, home);
        else throw std::runtime_error("Only ~ and ~/path are supported");
    }
    return p;
}

std::map<std::string, std::string> parse_conf(const std::string& path)
{
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open config: " + path);

    std::map<std::string, std::string> kv;
    std::string line;
    int ln = 0;
    while (std::getline(f, line)) {
        ++ln;
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        const auto eq = line.find('=');
        if (eq == std::string::npos)
            throw std::runtime_error(path + ":" + std::to_string(ln) + ": expected key=value");
        std::string key = trim(line.substr(0, eq));
        std::string val = trim(line.substr(eq + 1));
        if (key.empty())
            throw std::runtime_error(path + ":" + std::to_string(ln) + ": empty key");
        kv[key] = val;
    }
    return kv;
}

const std::string& require(const std::map<std::string, std::string>& kv, const std::string& k)
{
    const auto it = kv.find(k);
    if (it == kv.end() || it->second.empty())
        throw std::runtime_error("Missing required config key: " + k);
    return it->second;
}

std::string optional(const std::map<std::string, std::string>& kv,
                     const std::string& k, const std::string& def)
{
    const auto it = kv.find(k);
    return (it == kv.end() || it->second.empty()) ? def : it->second;
}

int parse_int(const std::string& s, const std::string& k)
{
    try { return std::stoi(s); }
    catch (...) { throw std::runtime_error("Invalid int for " + k + ": " + s); }
}

double parse_double(const std::string& s, const std::string& k)
{
    try { return std::stod(s); }
    catch (...) { throw std::runtime_error("Invalid number for " + k + ": " + s); }
}

bool parse_bool(const std::string& s, const std::string& k)
{
    if (s=="1"||s=="true"||s=="True"||s=="yes"||s=="on")  return true;
    if (s=="0"||s=="false"||s=="False"||s=="no"||s=="off") return false;
    throw std::runtime_error("Invalid bool for " + k + ": " + s);
}

bool file_ok(const std::string& p) { return !p.empty() && std::filesystem::is_regular_file(p); }

}  // namespace

SourceMode parse_source_mode(const std::string& v)
{
    if (v=="0"||v=="ros2")  return SourceMode::Ros2;
    if (v=="1"||v=="v4l2")  return SourceMode::V4l2;
    if (v=="2"||v=="video") return SourceMode::Video;
    throw std::runtime_error("Invalid source.mode: '" + v + "'. Use video|ros2|v4l2 (or 0/1/2)");
}

std::string source_label(const SourceConfig& source)
{
    switch (source.mode) {
    case SourceMode::Video: return "video";
    case SourceMode::V4l2:  return source.v4l2_device;
    case SourceMode::Ros2:  return source.ros2_topic;
    }
    return {};
}

VisionPilotConfig load_vision_pilot_config(const std::string& path)
{
    const auto kv = parse_conf(path);
    VisionPilotConfig cfg;

    cfg.engine.provider     = optional(kv, "engine.provider",     "cpu");
    cfg.engine.precision    = optional(kv, "model.precision",    "fp32");
    cfg.engine.device_id    = parse_int(optional(kv, "engine.device_id", "0"), "engine.device_id");
    cfg.engine.cache_dir    = expand_home(optional(kv, "engine.cache_dir", "/tmp/visionpilot_trt_cache"));
    cfg.engine.workspace_gb = parse_double(optional(kv, "engine.workspace_gb", "1.0"), "engine.workspace_gb");

    cfg.inference.precision    = optional(kv, "model.precision",    "fp32");

    cfg.source.mode          = parse_source_mode(optional(kv, "source.mode", "video"));
    cfg.source.video_path    = expand_home(optional(kv, "source.video_path", ""));
    cfg.source.video_realtime= parse_bool(optional(kv, "source.video_realtime", "true"), "source.video_realtime");
    cfg.source.video_loop    = parse_bool(optional(kv, "source.video_loop",     "false"), "source.video_loop");
    cfg.source.ros2_topic    = optional(kv, "source.ros2_topic",  "/camera/image");
    cfg.source.v4l2_device   = optional(kv, "source.v4l2_device", "/dev/video0");
    cfg.source.v4l2_fps      = parse_int(optional(kv, "source.v4l2_fps", "10"), "source.v4l2_fps");
    cfg.pipeline.initial_inference_check = parse_bool(
        optional(kv, "pipeline.initial_inference_check", "true"),
        "pipeline.initial_inference_check");

    cfg.fusion_debug = parse_bool(optional(kv, "fusion.debug", "false"), "fusion.debug");

    { const std::string raw = optional(kv, "debug.wheel_dir", "");
      cfg.wheel_dir = raw.empty() ? "" : expand_home(raw); }

    // Validate file paths
    if (cfg.source.mode == SourceMode::Video) {
        if (cfg.source.video_path.empty())
            throw std::runtime_error("source.mode=video requires source.video_path");
        if (!file_ok(cfg.source.video_path))
            throw std::runtime_error("source.video_path not found: " + cfg.source.video_path);
    }

    return cfg;
}

std::string resolve_vision_pilot_config_path(int argc, char** argv)
{
    for (int i = 1; i + 1 < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--config" || a == "-c") return argv[i + 1];
        const std::string pfx = "--config=";
        if (a.rfind(pfx, 0) == 0) return a.substr(pfx.size());
    }
    if (const char* e = std::getenv("VISIONPILOT_CONFIG"); e && file_ok(e)) return e;
    for (const char* c : {"config/vision_pilot.conf", "vision_pilot.conf"})
        if (file_ok(c)) return c;
    return {};
}
