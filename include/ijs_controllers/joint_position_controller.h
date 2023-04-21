// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

// #include <array>
// #include <string>
// #include <vector>

// #include <franka_hw/franka_cartesian_command_interface.h>
// #include <franka_hw/franka_model_interface.h>
// #include <franka_hw/franka_state_interface.h>
// #include <controller_interface/multi_interface_controller.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <hardware_interface/robot_hw.h>
// #include <ros/node_handle.h>
// #include <ros/time.h>
// #include <trajectory_msgs/JointTrajectoryPoint.h>

#include <memory>
#include <string>
#include <vector>

#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>
#include <franka_hw/franka_state_interface.h>

#include <Eigen/Dense>

namespace ijs_controllers {

class FrankaJointPosition : public controller_interface::MultiInterfaceController<
                                           hardware_interface::PositionJointInterface,
                                           franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  
  static constexpr double v_max{0.002};
  double period_{0.001};
  std::array<double, 7> q_target_;
  std::array<double, 7> dq_target_;
  
  ros::Subscriber sub_joint_command_;
  void jointCommandCallback(const trajectory_msgs::JointTrajectoryPointConstPtr& msg);

};

}  // namespace ijs_controllers
