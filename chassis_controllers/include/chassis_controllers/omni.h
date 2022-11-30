//
// Created by myx on 2022/12/1.
//
// ref: ref:https://github.com/rm-controls

#pragma once

#include <Eigen/Dense>

#include "chassis_base.h"

namespace chassis_controllers
{
class OmniController : public ChassisBase
{
public:
  OmniController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

private:
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;

  std::vector<std::shared_ptr<effort_controllers::JointVelocityController>> joints_;
  Eigen::MatrixXd chassis2joints_;
};

}  // namespace chassis_controllers