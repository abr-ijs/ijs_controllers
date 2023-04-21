// Copyright (c) 2017 Franka Emika GmbH
// Copyright (c) 2019 Jozef Stefan Institute

// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <ijs_controllers/joint_velocity_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace {

template <class T, size_t N>
std::ostream &operator<<(std::ostream &ostream, const std::array<T, N> &array)
{
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}

namespace ijs_controllers {

bool JointVelocity::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointVelocity: Error getting velocity joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointVelocity: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointVelocity: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointVelocity: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("JointVelocity: Could not get state interface from hardware");
    return false;
  }

  try {
    state_handle_.reset(
        new franka_hw::FrankaStateHandle(state_interface->getHandle("panda_robot")));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "JointVelocity: Exception getting state handle: " << e.what());
    return false;
  }

  // subscriber
  sub_joint_command_ = node_handle.subscribe(
      "command", 20, &JointVelocity::commandCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  return true;
}

void JointVelocity::starting(const ros::Time& /* time */) {
  ROS_INFO("Starting IJS Joint Velocity Controller"); 
  franka::RobotState robot_state = state_handle_->getRobotState();
  q_target_ = robot_state.q_d;
  q_d_ = robot_state.q_d;
  dq_target_ = {0}; 
  dq_d_ = {0}; 
}

void JointVelocity::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {

  // get last commanded configuration
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> last_q_target(robot_state.q_d);
  // ROS_DEBUG_STREAM("last_q_target: " << last_q_target);

  
  // update commanded states changed online by filtering
  for (size_t i = 0; i < 7; i++)
  {
    q_d_[i] = (1 - alpha) * q_target_[i] + alpha * q_d_[i];
    //q_d_[i] = q_target_[i];
    dq_d_[i] = (1 - alpha) * dq_target_[i] + alpha * dq_d_[i];
    //dq_d_[i] =  dq_target_[i];
  }

  // calculate configuration error and max component
  std::array<double, 7>  e_q, abs_e_q;
  for (size_t i = 0; i < 7; i++) {
   e_q[i] = (q_d_[i] - last_q_target[i])*5.; 
   abs_e_q[i] = std::abs(e_q[i]); 
  }
  double max_abs_e = *std::max_element(abs_e_q.begin(), abs_e_q.end());

  // calculate velocities from error and target velocities
  std::array<double, 7>  dq_calculated, abs_dq_calculated;
  for (size_t i = 0; i < 7; i++) {
   dq_calculated[i] = e_q[i]/std::max(max_abs_e,0.4)*0.4 + dq_target_[i]; 
   abs_dq_calculated[i] = std::abs(dq_calculated[i]);
  }

  // limit velocities to joint dq_max 
  double max_abs_dq= *std::max_element(abs_dq_calculated.begin(), abs_dq_calculated.end());
  for (size_t i = 0; i < 7; i++) {
   dq_calculated[i] = dq_calculated[i]/std::max(max_abs_dq/dq_max,1.); 
  }

  // send calculated velocities
  for (size_t i = 0; i < 7; ++i) {
    velocity_joint_handles_[i].setCommand(dq_calculated[i]);
  }
}

void JointVelocity::commandCallback(
    const robot_module_msgs::JointCommandConstPtr& msg) {
  
  // output new target configuration
  ROS_DEBUG("-----------------------------------------------------------------");
  ROS_DEBUG("positions : %6.2f, %6.2f, %6.2f, %6.2f %6.2f, %6.2f, %6.2f", msg->pos[0], msg->pos[1], msg->pos[2],msg->pos[3], msg->pos[4], msg->pos[5], msg->pos[6]);
  ROS_DEBUG("velocities: %6.2f, %6.2f, %6.2f, %6.2f %6.2f, %6.2f, %6.2f", msg->vel[0], msg->vel[1], msg->vel[2],msg->vel[3], msg->vel[4], msg->vel[5], msg->vel[6]);

  for (size_t i = 0; i < 7; i++) {
   q_target_[i] = msg->pos[i]; 
   dq_target_[i] = msg->vel[i]; 
  }
}

void JointVelocity::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace ijs_controllers

PLUGINLIB_EXPORT_CLASS(ijs_controllers::JointVelocity,
                       controller_interface::ControllerBase)
