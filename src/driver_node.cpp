/*
 * aubo_driver_node — minimal ROS2 bringup node
 *
 * In normal operation the hardware plugin is loaded automatically by
 * controller_manager via the ros2_control URDF tag — you do not need this
 * node for that path.
 *
 * This node is useful for:
 *   - Verifying the package builds and the plugin can be found
 *   - Running diagnostics without a full MoveIt / ros2_control stack
 *
 * Usage:
 *   ros2 run aubo_driver aubo_driver_node --ros-args -p robot_ip:=192.168.1.102
 */

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("aubo_driver_node");

    // In the ros2_control workflow the controller_manager loads the hardware
    // plugin from the URDF <ros2_control> tag.  Point it at the right config:
    //
    //   ros2 launch controller_manager controller_manager.launch.py \
    //       robot_description:="$(cat your_robot.urdf)"
    //
    // or use the provided aubo_i5_bringup.launch.py which does this for you.
    RCLCPP_INFO(node->get_logger(),
                "aubo_driver_node started. "
                "To control the robot, launch via aubo_i5_bringup.launch.py "
                "which will start controller_manager and load the hardware plugin.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
