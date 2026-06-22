#include <control_cmd_publisher/cmd_to_ros2.hpp>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <chrono>
#include <stdexcept>

namespace vp_middleware {

// ── helpers ───────────────────────────────────────────────────────────────────

static uint64_t ctrl_now_ns() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count());
}

// ── constructor ───────────────────────────────────────────────────────────────

ControlCmdPublisher::ControlCmdPublisher(
    const std::string& steering_topic,
    const std::string& throttle_topic,
    const std::string& node_name)
{
    // Init ROS2 once (camera_subscriber or vehicle_state_subscriber may have done it)
    if (!rclcpp::ok()) {
        static int    argc   = 1;
        static const char* fake_argv[] = {"control_cmd_publisher", nullptr};
        rclcpp::init(argc, const_cast<char**>(fake_argv));
    }

    node_ = std::make_shared<rclcpp::Node>(node_name);
    RCLCPP_INFO(node_->get_logger(),
                "ControlCmdPublisher: steering=%s  throttle=%s",
                steering_topic.c_str(), throttle_topic.c_str());

    // Reliable, depth=1 — commands must not be silently dropped
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    steering_pub_ = node_->create_publisher<std_msgs::msg::Float32>(steering_topic, qos);
    throttle_pub_ = node_->create_publisher<std_msgs::msg::Float32>(throttle_topic, qos);

    open_shm();
    RCLCPP_INFO(node_->get_logger(), "ControlCmdPublisher ready  (shmem: %s)", VP_CTRL_SHM_NAME);

    // Spin in background so that reliable QoS ACKs / discovery can proceed
    spin_thread_ = std::thread([this]() { rclcpp::spin(node_); });
}

// ── POSIX shmem ───────────────────────────────────────────────────────────────

void ControlCmdPublisher::open_shm() {
    shm_fd_ = shm_open(VP_CTRL_SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd_ < 0) {
        RCLCPP_WARN(node_->get_logger(), "shm_open(%s) failed — shmem disabled", VP_CTRL_SHM_NAME);
        return;
    }
    if (ftruncate(shm_fd_, sizeof(ControlCmdShmLayout)) < 0) {
        RCLCPP_WARN(node_->get_logger(), "ftruncate failed — shmem disabled");
        close(shm_fd_);
        shm_fd_ = -1;
        return;
    }
    shm_ptr_ = mmap(nullptr, sizeof(ControlCmdShmLayout),
                    PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if (shm_ptr_ == MAP_FAILED) {
        RCLCPP_WARN(node_->get_logger(), "mmap failed — shmem disabled");
        close(shm_fd_);
        shm_fd_  = -1;
        shm_ptr_ = nullptr;
    }
}

void ControlCmdPublisher::write_to_shm(float tyre_angle_rad, float acceleration_ms2) {
    if (!shm_ptr_) return;
    auto* layout = reinterpret_cast<ControlCmdShmLayout*>(shm_ptr_);

    // Seqlock write
    __atomic_fetch_add(&layout->epoch, 1u, __ATOMIC_RELEASE);
    layout->tyre_angle_rad    = tyre_angle_rad;
    layout->acceleration_ms2  = acceleration_ms2;
    layout->timestamp_ns      = ctrl_now_ns();
    __atomic_fetch_add(&layout->epoch, 1u, __ATOMIC_RELEASE);
}

// ── public API ────────────────────────────────────────────────────────────────

void ControlCmdPublisher::publish(float tyre_angle_rad, float acceleration_ms2) {
    // 1. ROS2
    std_msgs::msg::Float32 steer_msg;
    steer_msg.data = tyre_angle_rad;
    steering_pub_->publish(steer_msg);

    std_msgs::msg::Float32 throttle_msg;
    throttle_msg.data = acceleration_ms2;
    throttle_pub_->publish(throttle_msg);

    RCLCPP_DEBUG(node_->get_logger(),
                 "cmd: tyre=%.4f rad  accel=%.3f m/s²",
                 tyre_angle_rad, acceleration_ms2);

    // 2. POSIX shmem
    write_to_shm(tyre_angle_rad, acceleration_ms2);
}

// ── destructor ────────────────────────────────────────────────────────────────

ControlCmdPublisher::~ControlCmdPublisher() {
    if (shm_ptr_) munmap(shm_ptr_, sizeof(ControlCmdShmLayout));
    if (shm_fd_ >= 0) close(shm_fd_);
    if (spin_thread_.joinable()) spin_thread_.join();
}

}  // namespace vp_middleware
