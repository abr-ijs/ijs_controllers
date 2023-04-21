// Copyright (c) 2017 Franka Emika GmbH
// Copyright (c) 2019-2023 Jozef Stefan Institute
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <ijs_controllers/cartesian_impedance_controller.h>

#include <cmath>

#include <algorithm>
#include <array>
#include <cstring>
#include <iterator>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "pseudo_inversion.h"
#include "eigen_functions.h"

namespace
{

  template <class T, size_t N>
  std::ostream &operator<<(std::ostream &ostream, const std::array<T, N> &array)
  {
    ostream << "[";
    std::copy(array.cbegin(), array.cend() - 1,
              std::ostream_iterator<T>(ostream, ","));
    std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
    ostream << "]";
    return ostream;
  }
} // anonymous namespace

namespace ijs_controllers
{

  template <typename C>
  std::ostream &operator<<(std::ostream &os, const std::vector<C> &c)
  {
    for (const auto &v : c)
    {
      os << v << ", ";
    }
    return os;
  }

  bool CartesianImpedance::init(hardware_interface::RobotHW *robot_hw,
                                ros::NodeHandle &node_handle)
  {

    std::vector<double> cartesian_stiffness_vector;
    std::vector<double> cartesian_damping_vector;

    sub_command_ = node_handle.subscribe(
        "command", 20, &CartesianImpedance::commandCallback, this,
        ros::TransportHints().reliable().tcpNoDelay());

    sub_stiff_ = node_handle.subscribe(
        "stiffness", 20, &CartesianImpedance::stiffnessCallback, this,
        ros::TransportHints().reliable().tcpNoDelay());

    sub_nullspace_q_ = node_handle.subscribe(
        "nullspace_q", 5, &CartesianImpedance::nullspaceTargetCallback, this,
        ros::TransportHints().reliable().tcpNoDelay());

    sub_nullspace_stiff_ = node_handle.subscribe(
        "nullspace_stiff", 5, &CartesianImpedance::nullspaceStiffCallback, this,
        ros::TransportHints().reliable().tcpNoDelay());

    // reset_target_pose_service_ =
    // node_handle.advertiseService("reset_target_pose_to_current",  ... );
    reset_target_pose_service_ = node_handle.subscribe(
        "reset_target", 1, &CartesianImpedance::resetPoseTargetCallback, this,
        ros::TransportHints().reliable().tcpNoDelay());

    double publish_rate(100.0);
    if (!node_handle.getParam("publish_rate", publish_rate))
    {
      ROS_INFO_STREAM("CartesianImpedance: publish_rate not found. Defaulting to "
                      << publish_rate);
    }

    // filter from param server
    change_filter_param_ = node_handle.subscribe(
        "filter_pram", 1, &CartesianImpedance::changeFilterCallback, this,
        ros::TransportHints().reliable().tcpNoDelay());

    // init safety param override
    if (!node_handle.getParam("send_tau_immediately", send_tau_immediately_))
    {
      ROS_INFO_STREAM("CartesianImpedance: send_tau_immediately param not found. Defaulting to "
                      << send_tau_immediately_);
    }
    else
    {
      ROS_INFO_STREAM("CartesianImpedance: send_tau_immediately param is set to "
                      << send_tau_immediately_);
    }

    // treshold for pos/ori jumps
    if (!node_handle.getParam("max_position_step", MAX_POSITION_STEP_))
    {
      ROS_INFO_STREAM("CartesianImpedance: max_position_step param not found. Defaulting to "
                      << MAX_POSITION_STEP_);
    } else {
      ROS_INFO_STREAM("CartesianImpedance: max_position_step param is set to "
                      << MAX_POSITION_STEP_);
    }
    if (!node_handle.getParam("max_orientation_step", MAX_ORIENTATION_STEP_))
    {
      ROS_INFO_STREAM("CartesianImpedance: max_orientation_step param not found. Defaulting to "
                      << MAX_ORIENTATION_STEP_);
    } else {
      ROS_INFO_STREAM("CartesianImpedance: max_orientation_step param is set to "
                      << MAX_ORIENTATION_STEP_);
    }

    // arm id
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR_STREAM_NAMED("CartesianImpedance", "Could not read parameter arm_id");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) ||
        joint_names.size() != 7)
    {
      ROS_ERROR(
          "CartesianImpedance: Invalid or no joint_names parameters provided, "
          "aborting controller init!");
      return false;
    }

    franka_hw::FrankaModelInterface *model_interface =
        robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedance: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_.reset(new franka_hw::FrankaModelHandle(
          model_interface->getHandle(arm_id + "_model")));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedance: Exception getting model handle from interface: "
          << ex.what());
      return false;
    }

    franka_hw::FrankaStateInterface *state_interface =
        robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedance: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_.reset(new franka_hw::FrankaStateHandle(
          state_interface->getHandle(arm_id + "_robot")));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "CartesianImpedance: Exception getting state handle from interface: "
          << ex.what());
      return false;
    }

    hardware_interface::EffortJointInterface *effort_joint_interface =
        robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM_NAMED("CartesianImpedance", "Error getting effort joint interface "
                       "from hardware");
      return false;
    }
    for (size_t i = 0; i < 7; ++i)
    {
      try
      {
        joint_handles_.push_back(
            effort_joint_interface->getHandle(joint_names[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException &ex)
      {
        ROS_ERROR_STREAM_NAMED("CartesianImpedance", "Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    position_d_.setZero();
    orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;

    position_d_target_.setZero();
    orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

    cartesian_stiffness_.setZero();
    cartesian_damping_.setZero();
    cartesian_stiffness_target_.setZero();
    cartesian_damping_target_.setZero();

    nullspace_stiffness_.setZero();
    nullspace_stiffness_target_.setZero();

    ROS_INFO_STREAM_NAMED("CartesianImpedance", "Compiled at " << __DATE__ << ", " << __TIME__);

  rbs_interface_.reset(new rbs_interface::ProgramSafetyInterface(node_handle));

  return true;
}

  void CartesianImpedance::starting(const ros::Time & /*time*/)
  {

    // TODO: remove any commands on the subscribed topics!

    franka::RobotState initial_state = state_handle_->getRobotState();
    // convert to eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

    ROS_INFO("--------------------------------------------------");
    ROS_INFO("Initial state");
    ROS_INFO("--------------------------------------------------");
    ROS_INFO_STREAM("q_initial : " << q_initial.transpose());
    ROS_INFO_STREAM("dq_initial : " << dq_initial.transpose());

    // set equilibrium point to current state
    position_d_ = initial_transform.translation();
    orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
    position_d_target_ = initial_transform.translation();
    orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());
    velocity_d_.setZero();
    velocity_d_target_.setZero();
    wrench_d_.setZero();
    wrench_d_target_.setZero();

    ROS_INFO("pos initial : [%f,%f,%f]", position_d_(0), position_d_(1),
             position_d_(2));
    ROS_INFO("rot initial : %f [%f %f %f]", orientation_d_.w(), orientation_d_.x(),
             orientation_d_.y(), orientation_d_.z());

    if (send_tau_immediately_)
    {
      POWER_ENABLED_ = 1;
      ERR_FLAG_ = 0;
    }
    else
    {
    // enforce power on switch after each stop/start sequence
    POWER_ENABLED_ = 0;
    POWER_ENABLE_COUNT_ = 0;
    ERR_FLAG_ = 0;
    }

    // set nullspace equilibrium configuration to initial q
    q_d_nullspace_ = q_initial;
    q_d_nullspace_target_ = q_initial;

    nullspace_stiffness_ << 0,0,0,0,0,0;
    nullspace_stiffness_target_ << 0,0,0,0,0,0;

    // set initial gains
    cartesian_stiffness_target_.topLeftCorner(3, 3)
        << 100 * Eigen::Matrix3d::Identity();
    cartesian_stiffness_target_.bottomRightCorner(3, 3)
        << 10 * Eigen::Matrix3d::Identity();

    cartesian_damping_target_.topLeftCorner(3, 3)
        << 2.0 * sqrt(100) * Eigen::Matrix3d::Identity();
    cartesian_damping_target_.bottomRightCorner(3, 3)
        << 2.0 * sqrt(10) * Eigen::Matrix3d::Identity();
  }

  void CartesianImpedance::update(const ros::Time & /*time*/,
                                  const ros::Duration & /*period*/)
  {

    // get state variables
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    // convert to Eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
        robot_state.tau_J_d.data());
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.rotation());

    // compute difference to desired pose
    Eigen::Matrix<double, 6, 1> error;
    // positional part
    error.head(3) << position - position_d_;
    // orientation error
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0)
    {
      orientation.coeffs() << -orientation.coeffs();
    }
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
    Eigen::Vector3d lq(2 * q_log(error_quaternion));
    error.tail(3) << lq.x(), lq.y(), lq.z();
    // Transform to base frame
    error.tail(3) << -transform.rotation() * error.tail(3);
    
    // Calculate velocity error
    Eigen::Matrix<double, 6, 1> velocity_error(jacobian * dq - velocity_d_);

    // compute control
    Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_ft_added(7);

    // pseudoinverse for nullspace handling
    // kinematic pseuoinverse
    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    // Cartesian control
    tau_task << jacobian.transpose() * (-cartesian_stiffness_ * error - cartesian_damping_ * velocity_error);

    // additional user wrench
    tau_ft_added << jacobian.transpose() * wrench_d_;

    // torque based nullspace
    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv) * 
                      (nullspace_stiffness_.array() * (q_d_nullspace_ - q).array()).matrix();

    if (tau_nullspace.norm() > TAU_NULLSPACE_MAX_) {
      tau_nullspace = tau_nullspace*TAU_NULLSPACE_MAX_/tau_nullspace.norm();
    }

    // Desired torque
    tau_d << tau_task + tau_nullspace + coriolis + tau_ft_added;

    // Saturate torque rate to avoid discontinuities

    tau_d << saturateTorqueRate(tau_d, tau_J_d);

    check_and_send_tau(tau_d);
    

    // filter online parameters
    nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ +
                           (1.0 - filter_params_) * nullspace_stiffness_;
    cartesian_stiffness_ = filter_params_ * cartesian_stiffness_target_ + 
                           (1.0 - filter_params_) * cartesian_stiffness_;
    cartesian_damping_ = filter_params_ * cartesian_damping_target_ + 
                          (1.0 - filter_params_) * cartesian_damping_;
    position_d_ = filter_params_ * position_d_target_ +
                  (1.0 - filter_params_) * position_d_;
    velocity_d_ = filter_params_ * velocity_d_target_ +
                  (1.0 - filter_params_) * velocity_d_;
    wrench_d_ = filter_params_ * wrench_d_target_ +
                (1.0 - filter_params_) * wrench_d_;
    q_d_nullspace_ = filter_params_ * q_d_nullspace_target_ +
                     (1.0 - filter_params_) * q_d_nullspace_;

    orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

  }

void CartesianImpedance::check_and_send_tau(const Eigen::VectorXd& tau_d) {
    // torque has to be below TAU_POWER_ON_LIMIT_ for MAX_POWER_ENABLE_COUNT_
    // samples
    if (not(POWER_ENABLED_)) {
      double TauNorm = tau_d.norm();
      if (TauNorm < TAU_POWER_ON_LIMIT_) {
        POWER_ENABLE_COUNT_++;
      } else {
        POWER_ENABLE_COUNT_ = 0;
        ROS_ERROR_STREAM_ONCE_NAMED("CartesianImpedance","Can't power on, torque norm too big: " << TauNorm);
      }
    }

    // if torque constrains were fulfilled for MAX_POWER_ENABLE_COUNT_ consecutive cases,
    // start sending commands
    if (POWER_ENABLED_) {
      for (size_t i = 0; i < 7; ++i)
      {
        joint_handles_[i].setCommand(tau_d(i));
      }
    } else {
      if (POWER_ENABLE_COUNT_ >= MAX_POWER_ENABLE_COUNT_) {
        POWER_ENABLED_ = true;
        ROS_INFO_NAMED("CartesianImpedance","Power enabled");
      }
    }
}


  Eigen::Matrix<double, 7, 1> CartesianImpedance::saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>
          &tau_J_d)
  {
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++)
    {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] =
          tau_J_d[i] +
          std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    return tau_d_saturated;
  }

  void CartesianImpedance::resetPoseTargetCallback(
      const std_msgs::EmptyConstPtr &msg)
  {
    ROS_DEBUG("-------------------------------------------");

    // get robot state
    franka::RobotState robot_state = state_handle_->getRobotState();
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.rotation());

    // should we update position_d_  and orientation_d_ as well?
    orientation_d_target_ = orientation;
    position_d_target_ = position;

    ROS_DEBUG("reset pose to: (%f, %f, %f), %f <%f, %f, %f>",
              position_d_target_[0], position_d_target_[1], position_d_target_[2],
              orientation_d_target_.w(), orientation_d_target_.x(),
              orientation_d_target_.y(), orientation_d_target_.z());
  }

void CartesianImpedance::commandCallback(
    const robot_module_msgs::CartesianCommandConstPtr &msg) {

  if (rbs_interface_->isActive())
  {
    // Pose
    change_pose(msg->pose);

    // Velocity
    change_velocity(msg->vel);

    // End-effector wrench
    change_wrench(msg->ft);

    // Impedance
    change_stiffness(msg->impedance);
  } else {
    ROS_WARN("Not active");
  }
}

  void CartesianImpedance::change_pose(const geometry_msgs::Pose &pose)
  {
    // first save last target
    Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
    Eigen::Vector3d last_position_d_target(position_d_target_);

    ROS_DEBUG("-------------------------------------------");
    ROS_DEBUG("new pose: (%f, %f, %f), %f <%f, %f, %f>", pose.position.x,
              pose.position.y, pose.position.z, pose.orientation.w,
              pose.orientation.x, pose.orientation.y,
              pose.orientation.z);
    ROS_DEBUG("old pose: (%f, %f, %f), %f <%f, %f, %f>",
              last_position_d_target[0], last_position_d_target[1],
              last_position_d_target[2], last_orientation_d_target.w(),
              last_orientation_d_target.x(), last_orientation_d_target.y(),
              last_orientation_d_target.z());
    position_d_target_ << pose.position.x, pose.position.y, pose.position.z;
    orientation_d_target_.coeffs() << pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w;

    // orientation sign check
    if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) <
        0.0)
    {
      orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
    }

    // check whether position and rotation error are within limits
    double position_displacement =
        (position_d_target_ - last_position_d_target).norm();
        
    double orientation_displacement =
        2 * q_log(last_orientation_d_target.inverse() * orientation_d_target_).norm();

    // if bigger than predefined step, ignore new input and set error flag
    if (position_displacement > MAX_POSITION_STEP_ ||
        orientation_displacement > MAX_ORIENTATION_STEP_)
    {
      ERR_FLAG_ = 1;
      ROS_WARN("max reference step exceeded - pos: %f m, orientation: %f rad",
               position_displacement, orientation_displacement);
      position_d_target_ = last_position_d_target;
      orientation_d_target_ = last_orientation_d_target;
    }
    else
    {
      ERR_FLAG_ = 0;
    }
  }

  void CartesianImpedance::change_wrench(const geometry_msgs::Wrench &wrench)
  {
    // first save last target
    Eigen::VectorXd last_wrench_d_target(wrench_d_target_);

    // read new target
    wrench_d_target_ << wrench.force.x, wrench.force.y, wrench.force.z,
        wrench.torque.x, wrench.torque.y, wrench.torque.z;

    ROS_DEBUG("new wrench: (%f, %f, %f), (%f, %f, %f)", wrench_d_target_(0),
              wrench_d_target_(1), wrench_d_target_(2),
              wrench_d_target_(3), wrench_d_target_(4),
              wrench_d_target_(5));

    ROS_DEBUG("old wrench: (%f, %f, %f), (%f, %f, %f)", last_wrench_d_target(0), last_wrench_d_target(1), last_wrench_d_target(2),
              last_wrench_d_target(3), last_wrench_d_target(4), last_wrench_d_target(5));
  }

  void CartesianImpedance::change_velocity(const geometry_msgs::Twist &twist)
  {
    // first save last target
    Eigen::VectorXd last_velocity_d_target(velocity_d_target_);

    // read new target
    velocity_d_target_ << twist.linear.x, twist.linear.y, twist.linear.z,
        twist.angular.x, twist.angular.y, twist.angular.z;

    ROS_DEBUG("new vel: (%f, %f, %f), (%f, %f, %f)", velocity_d_target_(0),
              velocity_d_target_(1), velocity_d_target_(2),
              velocity_d_target_(3), velocity_d_target_(4),
              velocity_d_target_(5));

    ROS_DEBUG("old vel: (%f, %f, %f), (%f, %f, %f)", last_velocity_d_target(0), last_velocity_d_target(1), last_velocity_d_target(2),
              last_velocity_d_target(3), last_velocity_d_target(4), last_velocity_d_target(5));
  }

  void CartesianImpedance::stiffnessCallback(
      const robot_module_msgs::ImpedanceParametersConstPtr &impedance)
  {
    change_stiffness(*impedance);
  }

  void CartesianImpedance::change_stiffness(
      robot_module_msgs::ImpedanceParameters impedance)
  {

    int stride = impedance.n;

    if (stride != 9)
    {
      ROS_ERROR("CartesianImpedance: Invalid impedance parameters provided.");
    }
    else
    {
      std::vector<double> k = impedance.k;
      cartesian_stiffness_target_.setIdentity();
      // positional 3x3
      cartesian_stiffness_target_.topLeftCorner(3, 3) << k[0], k[1], k[2], k[3],
          k[4], k[5], k[6], k[7], k[8];
      // rotational 3x3
      cartesian_stiffness_target_.bottomRightCorner(3, 3) << k[0 + stride],
          k[1 + stride], k[2 + stride], k[3 + stride], k[4 + stride],
          k[5 + stride], k[6 + stride], k[7 + stride], k[8 + stride];

      std::vector<double> d = impedance.d;
      cartesian_damping_target_.setIdentity();
      // positional damping 3x3
      cartesian_damping_target_.topLeftCorner(3, 3) << d[0], d[1], d[2], d[3],
          d[4], d[5], d[6], d[7], d[8];
      // rotational damping 3x3
      cartesian_damping_target_.bottomRightCorner(3, 3) << d[0 + stride],
          d[1 + stride], d[2 + stride], d[3 + stride], d[4 + stride],
          d[5 + stride], d[6 + stride], d[7 + stride], d[8 + stride];

      ROS_DEBUG_STREAM("stiffness:  " << k);
      ROS_DEBUG_STREAM("damping:    " << d);
    }
  }

  void CartesianImpedance::nullspaceTargetCallback(
      const std_msgs::Float32MultiArrayConstPtr &msg)
  {
    q_d_nullspace_target_ << msg->data[0], msg->data[1], msg->data[2],
        msg->data[3], msg->data[4], msg->data[5], msg->data[6];
  }

  void CartesianImpedance::nullspaceStiffCallback(
      const std_msgs::Float32MultiArrayConstPtr &msg)
  {
    nullspace_stiffness_target_ << msg->data[0], msg->data[1], msg->data[2],
        msg->data[3], msg->data[4], msg->data[5], msg->data[6];
  }

  void CartesianImpedance::changeFilterCallback(
      const std_msgs::Float32ConstPtr &msg)
  {
    ROS_INFO("filter changed to: %f", msg->data);
    filter_params_ = msg->data;
  }

  Eigen::Vector3d CartesianImpedance::q_log(Eigen::Quaterniond qu)
  {
    qu.normalize();
    Eigen::Vector3d vec, u(qu.x(), qu.y(), qu.z());
    if (u.norm() < 1.0e-12)
    {
      for (int i = 0; i < 3; i++)
      {
        vec.data()[i] = 0;
      }
    }
    else
    {
      vec = acos(qu.w()) * u / u.norm();
    }
    return vec;
  }

} // namespace ijs_controllers

PLUGINLIB_EXPORT_CLASS(ijs_controllers::CartesianImpedance,
                       controller_interface::ControllerBase)
