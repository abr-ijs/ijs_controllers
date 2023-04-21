// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <robot_module_msgs/CartesianCommand.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>

#include <rosgraph_msgs/Log.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>
#include <boost/scoped_ptr.hpp>

#include <ijs_controllers/activate_trigger.hpp>



namespace ijs_controllers {

class CartesianImpedance : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  
  std::unique_ptr<rbs_interface::ProgramSafetyInterface> rbs_interface_;

  //position and orientation error 
  double MAX_POSITION_STEP_{0.05};
  double MAX_ORIENTATION_STEP_{0.1};
  int POWER_ENABLE_COUNT_{0};
  int MAX_POWER_ENABLE_COUNT_{1000};
  bool POWER_ENABLED_{0};
  double TAU_POWER_ON_LIMIT_{2};
  double TAU_NULLSPACE_MAX_{5};

  // External override for initialisation safety
  bool send_tau_immediately_{false};

  //double filter_params_{0.005};
  double filter_params_{0.05};


  //max tau change in one sample
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_target_;

  Eigen::Matrix<double, 7, 1> nullspace_stiffness_;
  Eigen::Matrix<double, 7, 1> nullspace_stiffness_target_;


  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;
  Eigen::Matrix<double,6,1> velocity_d_;
  Eigen::Matrix<double,6,1> velocity_d_target_;
  Eigen::Matrix<double,6,1> wrench_d_;
  Eigen::Matrix<double,6,1> wrench_d_target_;


  // check at init
  bool ERR_FLAG_{0};

  // Cartesian command subscriber
  ros::Subscriber sub_command_;
  void commandCallback(const robot_module_msgs::CartesianCommandConstPtr& msg);
  void change_pose(const geometry_msgs::Pose& pose);
  void change_velocity(const geometry_msgs::Twist& twist);
  void change_wrench(const geometry_msgs::Wrench& wrench);
  
  ros::Subscriber sub_stiff_;
  void stiffnessCallback(const robot_module_msgs::ImpedanceParametersConstPtr& msg);
  void change_stiffness(robot_module_msgs::ImpedanceParameters impedance);

  // change to service
  //ros::ServiceServer reset_target_pose_service_;
  ros::Subscriber reset_target_pose_service_;
  void resetPoseTargetCallback(const std_msgs::EmptyConstPtr& msg);
  ros::Subscriber change_filter_param_;
  void changeFilterCallback(const std_msgs::Float32ConstPtr& msg);


  // Nullspace
  ros::Subscriber sub_nullspace_q_;
  ros::Subscriber sub_nullspace_stiff_;
  void nullspaceTargetCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
  void nullspaceStiffCallback(const std_msgs::Float32MultiArrayConstPtr& msg);


  // dbg output
  realtime_tools::RealtimePublisher<PandaDbg> dbg_publisher_;
  franka_hw::TriggerRate rate_trigger_{1.0};
  void publishDbg(const franka::RobotState& robot_state);
  

  // helper
  Eigen::Vector3d q_log(Eigen::Quaterniond qu);

  void check_and_send_tau(const Eigen::VectorXd& tau_d);
};

}  // namespace ijs_controllers
