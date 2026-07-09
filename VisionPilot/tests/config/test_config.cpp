// Guards the config defaults: vehicle_acceleration_topic once defaulted to
// /vehicle/steering_cmd, interleaving steering and throttle on one topic.
#include <config/vision_pilot_config.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;

namespace
{

int g_failures = 0;

void check(const char* name, bool ok)
{
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_failures;
}

void write_file(const fs::path& p, const std::string& body)
{
    std::ofstream f(p);
    f << body;
}

}  // namespace

int main()
{
    // Stage a minimal conf tree and chdir to it: find_config() reads config/<file>
    // relative to the CWD. vision_pilot_ros2.conf is left empty so every ROS2
    // key falls back to its compiled-in default.
    const fs::path root = fs::temp_directory_path() / "vp_test_config";
    fs::remove_all(root);
    fs::create_directories(root / "config");
    write_file(root / "config" / "vision_pilot.conf",
               "source.mode = ros2\n"
               "speed_limit = 10.0\n"
               "Lf = 2.67\n");
    write_file(root / "config" / "vision_pilot_ros2.conf", "");
    fs::current_path(root);

    const Config cfg = load_vision_pilot_config();

    check("source.mode parsed as ros2", cfg.source.mode == SourceMode::Ros2);
    check("speed_limit parsed", cfg.speed_limit == 10.0);

#ifdef ENABLE_ROS2_INTERFACE
    check("camera topic default", cfg.source.input_camera_topic == "/camera/image");
    check("speed topic default", cfg.vehicle_speed_topic == "/vehicle/speed");
    check("steering topic default", cfg.vehicle_steering_topic == "/vehicle/steering_cmd");
    check("acceleration topic default", cfg.vehicle_acceleration_topic == "/vehicle/throttle_cmd");
    check("steering and acceleration topics differ",
          cfg.vehicle_steering_topic != cfg.vehicle_acceleration_topic);
#endif

    std::printf("\n%s (%d failure%s)\n", g_failures ? "FAILED" : "ALL PASS", g_failures,
                g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 1;
}
