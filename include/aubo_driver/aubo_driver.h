#ifndef AUBO_DRIVER_H_
#define AUBO_DRIVER_H_

#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Old firmware SDK headers (unchanged — these live in include/aubo_driver/)
#include "aubo_driver/AuboRobotMetaType.h"
#include "aubo_driver/serviceinterface.h"

namespace aubo_driver {

/**
 * ros2_control SystemInterface plugin for Aubo robots running the legacy
 * firmware (pre-RTDE).  Communication uses the old SDK:
 *   - Two ServiceInterface connections on port 8899 (send + receive)
 *   - robotServiceEnterTcp2CanbusMode() for position streaming
 *   - robotServiceSetRobotPosData2Canbus() to push joint commands
 *   - robotServiceGetCurrentWaypointInfo() to read joint state (background thread)
 *
 * Hardware parameters (from ros2_control URDF tag):
 *   robot_ip   IP address of the robot controller  (default: 127.0.0.1)
 */
class AuboHardwareInterface : public hardware_interface::SystemInterface
{
public:
    AuboHardwareInterface() = default;
    ~AuboHardwareInterface() override;

    // --- Lifecycle callbacks --------------------------------------------------

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    // --- ros2_control interface -----------------------------------------------

    std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:
    // --- helpers --------------------------------------------------------------
    bool connectToRobot();
    void disconnectFromRobot();
    void pollThread();  // background thread: reads joint state at ~500 Hz

    // --- old SDK objects ------------------------------------------------------
    // Two separate connections: one dedicated to sending commands, one to
    // receiving state — same pattern as the original ROS1 driver.
    ServiceInterface robot_send_service_;
    ServiceInterface robot_receive_service_;

    // --- state buffers --------------------------------------------------------
    // actual_q_raw_: written by pollThread under mtx_
    // actual_q_     : written by read() (copy from raw); StateInterfaces point here
    // cmd_q_        : written by ros2_control; CommandInterfaces point here
    static constexpr int DOF = 6;
    std::array<double, DOF> actual_q_raw_{};
    std::array<double, DOF> actual_q_{};
    std::array<double, DOF> cmd_q_{};

    std::mutex mtx_;

    // --- background poll thread -----------------------------------------------
    std::thread       poll_thread_;
    std::atomic<bool> poll_running_{false};

    // --- connection state -----------------------------------------------------
    std::atomic<bool> connected_{false};

    // --- config ---------------------------------------------------------------
    std::string robot_ip_{"127.0.0.1"};
    bool        initialized_{false};
};

}  // namespace aubo_driver

#endif  // AUBO_DRIVER_H_
