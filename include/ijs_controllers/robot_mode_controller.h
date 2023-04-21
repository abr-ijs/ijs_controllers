// Copyright (c) 2017 Franka Emika GmbH
// Copyright (c) 2019 Jozef Stefan Institute

// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <std_msgs/String.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>
#include <boost/scoped_ptr.hpp>
#include <realtime_tools/realtime_publisher.h>

namespace ijs_controllers {

class RobotMode : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;

  // Realtime publisher for errors
  franka_hw::TriggerRate rate_trigger_{1.0};
  realtime_tools::RealtimePublisher<std_msgs::String> dbg_publisher_;

};

}  // namespace ijs_controllers
