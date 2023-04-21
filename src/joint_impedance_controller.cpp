// Copyright (c) 2017 Franka Emika GmbH
// Copyright (c) 2019 Jozef Stefan Institute
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <ijs_controllers/joint_impedance_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace
{

template <class T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &input)
{
  for (auto const &i : input)
  {
    os << i << ", ";
  }
  return os;
}

template <class T, size_t N>
std::ostream &operator<<(std::ostream &ostream, const std::array<T, N> &array)
{
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}

} // namespace

namespace ijs_controllers
{

bool JointImpedance::init(hardware_interface::RobotHW *robot_hw,
                          ros::NodeHandle &node_handle)
{
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id))
  {
    ROS_ERROR("JointImpedance: Could not read parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
  {
    ROS_ERROR(
        "JointImpedance: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7)
  {
    ROS_ERROR(
        "JointImpedance:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }
  else
  {
    node_handle.getParam("k_gains", k_gains_target_);
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7)
  {
    ROS_ERROR(
        "JointImpedance:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }
  else
  {
    node_handle.getParam("d_gains", d_gains_target_);
  }

  reset_target_service_ = node_handle.subscribe(
      "reset_target", 1, &JointImpedance::resetTargetCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_joint_command_ = node_handle.subscribe(
      "command", 20, &JointImpedance::jointCommandCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  franka_hw::FrankaModelInterface *model_interface =
      robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr)
  {
    ROS_ERROR_STREAM(
        "JointImpedance: Error getting model interface from hardware");
    return false;
  }
  try
  {
    model_handle_.reset(
        new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
  }
  catch (hardware_interface::HardwareInterfaceException &ex)
  {
    ROS_ERROR_STREAM(
        "JointImpedance: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  franka_hw::FrankaStateInterface *state_interface =
      robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr)
  {
    ROS_ERROR_STREAM(
        "JointImpedance: Error getting state interface from hardware");
    return false;
  }
  try
  {
    state_handle_.reset(
        new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
  }
  catch (hardware_interface::HardwareInterfaceException &ex)
  {
    ROS_ERROR_STREAM(
        "JointImpedance: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  hardware_interface::EffortJointInterface *effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr)
  {
    ROS_ERROR_STREAM(
        "JointImpedance: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i)
  {
    try
    {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "JointImpedance: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  //initialize arrays
  std::fill(q_d_.begin(), q_d_.end(), 0);
  std::fill(dq_d_.begin(), dq_d_.end(), 0);
  std::fill(q_d_target_.begin(), q_d_target_.end(), 0);
  std::fill(dq_d_target_.begin(), dq_d_target_.end(), 0);
  std::fill(feed_forward_q_target_.begin(), feed_forward_q_target_.end(), 0);

  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);

  return true;
}

void JointImpedance::starting(const ros::Time & /*time*/)
{

  ROS_INFO("Starting IJS Joint Impedance Controller with feed-forward torque term");

  // initial desired position is existing joint space configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  q_d_ = initial_state.q;
  //q_d_target_ = initial_state.q_d;
  //dq_d_ = initial_state.dq_d;
  //dq_d_target_ = initial_state.dq_d;

  q_d_target_ = initial_state.q;
  std::fill(dq_d_.begin(), dq_d_.end(), 0);
  std::fill(dq_d_target_.begin(), dq_d_target_.end(), 0);

  //ROS_INFO_STREAM("-----------");
  //ROS_INFO_STREAM(" FCI q_msr : " << initial_state.q);
  //ROS_INFO_STREAM(" FCI q_des : " << initial_state.q_d);
  //ROS_INFO_STREAM("-----------");
}

void JointImpedance::update(const ros::Time & /*time*/,
                            const ros::Duration &period)
{

  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();

  std::array<double, 7> tau_d_calculated;
  for (size_t i = 0; i < 7; ++i)
  {
    tau_d_calculated[i] = coriolis[i] +
                          k_gains_[i] * (q_d_[i] - robot_state.q[i]) +
                          d_gains_[i] * (dq_d_[i] - robot_state.dq[i]) +
                          feed_forward_q_target_[i];
  }

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

  // Commanded torque has to be below 1 for MAX_POWER_ENABLE_COUNT_ samples
  Eigen::VectorXd tau(7);
  tau << tau_d_saturated[0], tau_d_saturated[1], tau_d_saturated[2],
      tau_d_saturated[3], tau_d_saturated[4], tau_d_saturated[5], tau_d_saturated[6];
  double TauNorm = tau.norm();

  if (not(POWER_ENABLED_))
  {
    if (TauNorm < 1.1)
    {
      POWER_ENABLE_COUNT_++;
    }
    else
    {
      POWER_ENABLE_COUNT_ = 0;
      ROS_ERROR_STREAM_ONCE("Can't power on, torque norm too big: " << TauNorm);
      ROS_DEBUG_STREAM_ONCE("  tau_saturated: " << tau_d_saturated);
      ROS_DEBUG_STREAM_ONCE("  tau_calculated: " << tau_d_calculated);
      ROS_DEBUG_STREAM_ONCE("  k_gains: " << k_gains_);
      ROS_DEBUG_STREAM_ONCE("  d_gains: " << d_gains_);
      ROS_DEBUG_STREAM_ONCE("  q_d_: " << q_d_);
      ROS_DEBUG_STREAM_ONCE("  dq_d_: " << dq_d_);
      ROS_DEBUG_STREAM_ONCE("  q_msr: " << robot_state.q);
      ROS_DEBUG_STREAM_ONCE("  dq_msr: " << robot_state.dq);
      ROS_DEBUG_STREAM_ONCE("  coriolis: " << coriolis);
    }
  }

  // if torque constrains were fuilfilled for MAX_POWER_ENABLE_COUNT_ consequitive cases,
  // start sending commands
  if (POWER_ENABLED_)
  {
    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(tau_d_saturated[i]);
    }
  }
  else
  {
    if (POWER_ENABLE_COUNT_ >= MAX_POWER_ENABLE_COUNT_)
    {
      POWER_ENABLED_ = true;
      ROS_INFO_STREAM("Power enabled, torque norm: " << TauNorm);
      ROS_INFO_STREAM("period is " << period.toSec() << " ms");
    }
  }

  // update parameters changed online by filtering
  for (size_t i = 0; i < 7; i++)
  {
    k_gains_[i] = (1 - alpha) * k_gains_target_[i] + alpha * k_gains_[i];
    d_gains_[i] = (1 - alpha) * d_gains_target_[i] + alpha * d_gains_[i];
    q_d_[i] = (1 - alpha) * q_d_target_[i] + alpha * q_d_[i];
    //q_d_[i] = q_d_target_[i];
    dq_d_[i] = (1 - alpha) * dq_d_target_[i] + alpha * dq_d_[i];
    //dq_d_[i] =  dq_d_target_[i];
  }
}

std::array<double, 7> JointImpedance::saturateTorqueRate(
    const std::array<double, 7> &tau_d_calculated,
    const std::array<double, 7> &tau_J_d)
{ // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++)
  {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

void JointImpedance::jointCommandCallback(
    const robot_module_msgs::JointCommandConstPtr &msg)
{

  // first save last configuration
  //franka::RobotState robot_state = state_handle_->getRobotState();
  //std::array<double, 7> last_q_target(robot_state.q_d); desired se ne updajta, preveri ali je to vedno ko ne uporabljaš internih kontrolerjev
  
  std::array<double, 7> last_q_target = q_d_target_;

  // read new target
  ROS_DEBUG("-------------------------------------");
  ROS_DEBUG("positions: %6.2f, %6.2f, %6.2f, %6.2f %6.2f, %6.2f, %6.2f", msg->pos[0], msg->pos[1], msg->pos[2], msg->pos[3], msg->pos[4], msg->pos[5], msg->pos[6]);
  //ROS_DEBUG_STREAM("last_q_target: " << last_q_target);
  ROS_DEBUG("velocities: %6.2f, %6.2f, %6.2f, %6.2f %6.2f, %6.2f, %6.2f", msg->vel[0], msg->vel[1], msg->vel[2],msg->vel[3], msg->vel[4], msg->vel[5], msg->vel[6]);
  ROS_DEBUG("torque:    %6.2f, %6.2f, %6.2f, %6.2f %6.2f, %6.2f, %6.2f", msg->trq[0], msg->trq[1], msg->trq[2], msg->trq[3], msg->trq[4], msg->trq[5], msg->trq[6]);

  for (size_t i = 0; i < 7; i++)
  {
    q_d_target_[i] = msg->pos[i];
    dq_d_target_[i] =   msg->vel[i];  // (q_d_target_[i] - last_q_target[i]) * period_; //
    feed_forward_q_target_[i] = msg->trq[i];
  }
  /*if (std::any_of(dq_d_target_.begin(), dq_d_target_.end(), [](double vel) { return std::fabs(vel) > v_max; }))
  {
    ROS_DEBUG_STREAM("dq_d_target_: " << dq_d_target_);
    ROS_WARN("max dq exceeded, command ignored");
    q_d_target_ = last_q_target;
    // set target velocity to 0
    std::fill(dq_d_target_.begin(), dq_d_target_.end(), 0);
  }*/

  if ( msg->impedance.n != 7)
  {
    ROS_ERROR("JointImpedance: Invalid impedance parameters provided.");
  } 
  else
  {
    k_gains_target_ = msg->impedance.k;
    d_gains_target_ = msg->impedance.d;
    ROS_DEBUG_STREAM("stiffness:  " << k_gains_target_);
    ROS_DEBUG_STREAM("damping:    " << d_gains_target_);
  }
  
}

void JointImpedance::resetTargetCallback(
    const std_msgs::EmptyConstPtr &msg)
{
  franka::RobotState robot_state = state_handle_->getRobotState();
  q_d_target_ = robot_state.q;
  dq_d_target_ = robot_state.dq;
  std::fill(feed_forward_q_target_.begin(), feed_forward_q_target_.end(), 0);
  ROS_DEBUG("-------------------------------------");
  ROS_DEBUG("Desired configuration set to actual configuration, torques set to zero");
}

} // namespace ijs_controllers

PLUGINLIB_EXPORT_CLASS(ijs_controllers::JointImpedance,
                       controller_interface::ControllerBase)
