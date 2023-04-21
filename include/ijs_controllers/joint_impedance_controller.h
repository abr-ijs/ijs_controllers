// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
//#pragma once

#include <memory>
#include <string>
#include <vector>

#include <robot_module_msgs/JointCommand.h>
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

class JointImpedance : public controller_interface::MultiInterfaceController<
                                            franka_hw::FrankaModelInterface,
                                            hardware_interface::EffortJointInterface,
                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  std::array<double, 7> saturateTorqueRate(
      const std::array<double, 7>& tau_d_calculated,
      const std::array<double, 7>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  static constexpr double kDeltaTauMax{1.0};
  static constexpr double v_max{0.002};
  //static constexpr double v_max{0.05};
  int POWER_ENABLE_COUNT_{0};
  int MAX_POWER_ENABLE_COUNT_{1000};
  bool POWER_ENABLED_{0};
  double period_{0.001};

  std::vector<double> k_gains_;
  std::vector<double> d_gains_;
  std::array<double, 7> dq_filtered_;

  //std::array<double, 16> initial_pose_;
  std::array<double, 7> q_d_;
  std::array<double, 7> dq_d_;

  double alpha{0.99};
  std::vector<double> k_gains_target_;
  std::vector<double> d_gains_target_;
  std::array<double, 7> q_d_target_;
  std::array<double, 7> dq_d_target_;
  std::array<double, 7> feed_forward_q_target_;

  franka_hw::TriggerRate rate_trigger_{1.0};
  std::array<double, 7> last_tau_d_{};
  //realtime_tools::RealtimePublisher<JointTorqueComparison> torques_publisher_;

  ros::Subscriber sub_joint_command_;
  void jointCommandCallback(const robot_module_msgs::JointCommandConstPtr& msg);

  ros::Subscriber reset_target_service_;
  void resetTargetCallback(const std_msgs::EmptyConstPtr& msg);

  //ros::Subscriber sub_stiffness_;
  //void complianceParamCallback(const std_msgs::Float32MultiArrayConstPtr& msg);


};

}   
