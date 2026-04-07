/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, AUBO Robotics
 * All rights reserved.
 *
 * ROS2 port: wraps the legacy firmware SDK (ServiceInterface / Tcp2CanbusMode)
 * inside a ros2_control hardware_interface::SystemInterface plugin.
 * The robot-side protocol is unchanged from the ROS1 driver — only the
 * ROS framework layer has been updated.
 */

#include "aubo_driver/aubo_driver.h"

#include <chrono>
#include <pluginlib/class_list_macros.hpp>

namespace aubo_driver {

// ---------------------------------------------------------------------------
// Destructor
// ---------------------------------------------------------------------------

AuboHardwareInterface::~AuboHardwareInterface()
{
    disconnectFromRobot();
}

// ---------------------------------------------------------------------------
// on_init  — validate URDF hardware config, read parameters
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn AuboHardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Read robot IP from the <hardware><param name="robot_ip"> URDF tag
    if (info_.hardware_parameters.count("robot_ip")) {
        robot_ip_ = info_.hardware_parameters.at("robot_ip");
    }
    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
                "Robot IP: %s", robot_ip_.c_str());

    // Validate each joint: exactly 1 position command interface,
    // at least 1 position state interface.
    for (const auto & joint : info_.joints) {
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("AuboHardwareInterface"),
                "Joint '%s' has %zu command interface(s), 1 (position) expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("AuboHardwareInterface"),
                "Joint '%s' command interface is '%s', 'position' expected.",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces.empty()) {
            RCLCPP_FATAL(rclcpp::get_logger("AuboHardwareInterface"),
                "Joint '%s' has no state interfaces.", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("AuboHardwareInterface"),
                "Joint '%s' first state interface is '%s', 'position' expected.",
                joint.name.c_str(), joint.state_interfaces[0].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_activate  — connect to robot, enter Tcp2CanbusMode, start poll thread
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn AuboHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
                "Activating — connecting to robot at %s ...", robot_ip_.c_str());

    if (!connectToRobot()) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Seed command positions from the current robot position so the robot
    // does not jump when the controller first takes over.
    aubo_robot_namespace::wayPoint_S wp;
    int ret = robot_receive_service_.robotServiceGetCurrentWaypointInfo(wp);
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
        std::lock_guard<std::mutex> lk(mtx_);
        for (int i = 0; i < DOF; i++) {
            actual_q_raw_[i] = wp.jointpos[i];
            actual_q_[i]     = wp.jointpos[i];
            cmd_q_[i]        = wp.jointpos[i];
        }
        initialized_ = true;
    } else {
        RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                    "Could not read initial joint positions (ret=%d). "
                    "Commands will start at zero.", ret);
    }

    // Launch the background thread that polls joint state at ~500 Hz.
    poll_running_ = true;
    poll_thread_ = std::thread(&AuboHardwareInterface::pollThread, this);

    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
                "Activation complete.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_deactivate  — stop poll thread, leave Tcp2CanbusMode, logout
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn AuboHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"), "Deactivating...");
    disconnectFromRobot();
    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"), "Deactivated.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// export_state_interfaces  — position per joint (actual_q_ array)
// ---------------------------------------------------------------------------

std::vector<hardware_interface::StateInterface>
AuboHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> si;
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        si.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_POSITION,
                        &actual_q_[i]);
    }
    return si;
}

// ---------------------------------------------------------------------------
// export_command_interfaces  — position per joint (cmd_q_ array)
// ---------------------------------------------------------------------------

std::vector<hardware_interface::CommandInterface>
AuboHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ci;
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        ci.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_POSITION,
                        &cmd_q_[i]);
    }
    return ci;
}

// ---------------------------------------------------------------------------
// read  — copy latest poll-thread snapshot into the StateInterface buffers
// ---------------------------------------------------------------------------

hardware_interface::return_type AuboHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // actual_q_raw_ is written by pollThread under mtx_.
    // Copy it into actual_q_ (which StateInterfaces point to) so the
    // controller_manager sees a consistent snapshot each control cycle.
    std::lock_guard<std::mutex> lk(mtx_);
    actual_q_ = actual_q_raw_;
    return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// write  — send joint position command to robot via Tcp2CanbusMode
// ---------------------------------------------------------------------------

hardware_interface::return_type AuboHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!connected_) {
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("AuboHardwareInterface"),
            *rclcpp::Clock::make_shared(), 2000,
            "write() called but robot is not connected.");
        return hardware_interface::return_type::ERROR;
    }

    double joints[DOF];
    for (int i = 0; i < DOF; i++) joints[i] = cmd_q_[i];

    int ret = robot_send_service_.robotServiceSetRobotPosData2Canbus(joints);
    if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("AuboHardwareInterface"),
            *rclcpp::Clock::make_shared(), 1000,
            "robotServiceSetRobotPosData2Canbus failed (ret=%d).", ret);
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// connectToRobot  — login x2, enter Tcp2CanbusMode
// ---------------------------------------------------------------------------

bool AuboHardwareInterface::connectToRobot()
{
    const int max_tries = 5;

    // --- send service login ---
    int ret = aubo_robot_namespace::ErrCode_SocketDisconnect;
    for (int i = 0; i < max_tries && ret != aubo_robot_namespace::InterfaceCallSuccCode; i++) {
        ret = robot_send_service_.robotServiceLogin(
            robot_ip_.c_str(), 8899, "aubo", "123456");
        if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
            RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                        "Send-service login attempt %d/%d failed (ret=%d).",
                        i + 1, max_tries, ret);
        }
    }
    if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                     "Could not log in (send service) after %d attempts.", max_tries);
        return false;
    }

    // --- receive service login ---
    ret = robot_receive_service_.robotServiceLogin(
        robot_ip_.c_str(), 8899, "aubo", "123456");
    if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                     "Receive-service login failed (ret=%d).", ret);
        robot_send_service_.robotServiceLogout();
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"), "Logged in successfully.");

    // --- enter Tcp2CanbusMode so we can stream joint positions ---
    ret = robot_send_service_.robotServiceEnterTcp2CanbusMode();
    if (ret == aubo_robot_namespace::ErrCode_ResponseReturnError) {
        // Already in canbus mode from a previous session; leave then re-enter.
        robot_send_service_.robotServiceLeaveTcp2CanbusMode();
        ret = robot_send_service_.robotServiceEnterTcp2CanbusMode();
    }
    if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                     "Failed to enter Tcp2CanbusMode (ret=%d). "
                     "Is another client already in control?", ret);
        robot_receive_service_.robotServiceLogout();
        robot_send_service_.robotServiceLogout();
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
                "Entered Tcp2CanbusMode — ready for position streaming.");
    connected_ = true;
    return true;
}

// ---------------------------------------------------------------------------
// disconnectFromRobot  — stop poll thread, leave Tcp2CanbusMode, logout
// ---------------------------------------------------------------------------

void AuboHardwareInterface::disconnectFromRobot()
{
    // Stop the background poll thread first.
    poll_running_ = false;
    if (poll_thread_.joinable()) {
        poll_thread_.join();
    }

    if (connected_) {
        robot_send_service_.robotServiceLeaveTcp2CanbusMode();
        robot_receive_service_.robotServiceLogout();
        robot_send_service_.robotServiceLogout();
        connected_ = false;
        RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
                    "Disconnected from robot.");
    }
}

// ---------------------------------------------------------------------------
// pollThread  — background joint-state reader (~500 Hz)
// ---------------------------------------------------------------------------

void AuboHardwareInterface::pollThread()
{
    aubo_robot_namespace::wayPoint_S wp;

    while (poll_running_) {
        int ret = robot_receive_service_.robotServiceGetCurrentWaypointInfo(wp);

        if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
            std::lock_guard<std::mutex> lk(mtx_);
            for (int i = 0; i < DOF; i++) {
                actual_q_raw_[i] = wp.jointpos[i];
            }
        } else if (ret == aubo_robot_namespace::ErrCode_SocketDisconnect) {
            RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                         "Poll thread: socket disconnected.");
            connected_ = false;
            break;
        }
        // ~500 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

}  // namespace aubo_driver

// ---------------------------------------------------------------------------
// Pluginlib export — allows controller_manager to load this as a plugin
// ---------------------------------------------------------------------------
PLUGINLIB_EXPORT_CLASS(aubo_driver::AuboHardwareInterface,
                       hardware_interface::SystemInterface)
