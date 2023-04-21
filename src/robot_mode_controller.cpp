// Copyright (c) 2017 Franka Emika GmbH
// Copyright (c) 2019 Jozef Stefan Institute
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <ijs_controllers/robot_mode_controller.h>

#include <cstring>
#include <iterator>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace ijs_controllers {

bool RobotMode::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  
  double publish_rate(100.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("RobotMode: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("RobotMode: Could not read parameter arm_id");
    return false;
  }

  franka_hw::FrankaStateInterface* state_interface =
      robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "RobotMode: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_.reset(
        new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "RobotMode: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  // Realtime publisher
  dbg_publisher_.init(node_handle, "mode", 1);

  return true;
}

void RobotMode::starting(const ros::Time& /*time*/) {
  ROS_INFO_STREAM("Robot mode real-time publisher started");
}

void RobotMode::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {

    franka::RobotState robot_state = state_handle_->getRobotState();

    if (rate_trigger_() && dbg_publisher_.trylock()) {
        switch (robot_state.robot_mode) {
            case (franka::RobotMode::kUserStopped):
            dbg_publisher_.msg_.data =  "User stopped";
            break;
            case (franka::RobotMode::kIdle):
            dbg_publisher_.msg_.data =  "Idle";
            break;
            case (franka::RobotMode::kMove):
            dbg_publisher_.msg_.data =  "Move";
            break;
            case (franka::RobotMode::kGuiding):
            dbg_publisher_.msg_.data =  "Guiding";
            break;
            case (franka::RobotMode::kReflex):
            dbg_publisher_.msg_.data =  "Reflex";
            break;
            case (franka::RobotMode::kAutomaticErrorRecovery):
            dbg_publisher_.msg_.data =  "Automatic error recovery";
            break;
            case (franka::RobotMode::kOther):
            dbg_publisher_.msg_.data =  "Other";
            break;
        }
        dbg_publisher_.unlockAndPublish();
  }
  
}

}  // namespace ijs_controllers

PLUGINLIB_EXPORT_CLASS(ijs_controllers::RobotMode,
                       controller_interface::ControllerBase)
