#ifndef VISIONPILOT_CMD_TO_ROS2_HPP
#define VISIONPILOT_CMD_TO_ROS2_HPP

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include <control/control_command.hpp>

namespace control_cmd_publisher {

    /**
    * @class ControlCmdPublisher
    * @brief ROS2 node that publishes the agnostic ControlCommand as
    *        ackermann_msgs/AckermannDriveStamped on a configurable topic.
    *
    * The mapping is 1:1 (steering rad, speed m/s, acceleration m/s^2), so this is the
    * pure adapter for simulators / Ackermann-driven vehicles (issue #288); the DBW/CAN
    * path is separate (#287, the DbwAdapter seam).
    *
    * Context ownership: rclcpp is initialized only if no one else has (owns_init_), and
    * shut down on destruction only if this object owns it. This lets the publisher
    * coexist with the camera subscriber (which owns init/shutdown) without either node
    * tearing down the other's context.
    */
    class ControlCmdPublisher {

        public:

            /**
            * @brief Constructor.
            *
            * @param topic     ROS2 topic to publish on (e.g. "/control/ackermann_cmd").
            * @param frame_id  header.frame_id stamped on every message.
            * @param node_name Internal ROS2 node name.
            */
            explicit ControlCmdPublisher(
                const std::string& topic,
                const std::string& frame_id = "base_link",
                const std::string& node_name = "vision_pilot_control"
            );

            /**
            * @brief Destructor. Shuts down rclcpp only if this object initialized it.
            */
            ~ControlCmdPublisher();

            /**
            * @brief Publish one ControlCommand as an AckermannDriveStamped.
            */
            void publish(const ControlCommand& cmd);

        private:
            bool owns_init_ = false;
            std::string frame_id_;
            std::shared_ptr<rclcpp::Node> node_;
            rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_;
    };

}  // namespace control_cmd_publisher

#endif //VISIONPILOT_CMD_TO_ROS2_HPP
