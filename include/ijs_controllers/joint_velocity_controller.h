// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>

#include <robot_module_msgs/JointCommand.h>
#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace ijs_controllers {

class JointVelocity : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

 private:
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;

  double alpha{0.99};
  std::array<double, 7> dq_d_;
  std::array<double, 7> q_d_;
  std::array<double, 7> dq_target_;
  std::array<double, 7> q_target_;

  double dq_max = 2; //{2.1750,	2.1750,	2.1750,	2.1750,	2.6100, 2.6100,	2.6100};

  ros::Subscriber sub_joint_command_;
  void commandCallback(const robot_module_msgs::JointCommandConstPtr& msg);
};

}  // namespace ijs_controllers
