//
// Created by myx on 2022/11/15.
//
// ref:https://github.com/rm-controls
#pragma once

#include "chassis_base.h"

#include <rc_common/eigen_types.h>
#include <effort_controllers/joint_position_controller.h>

namespace chassis_controllers
{
struct Module
{
  Vec2<double> position_;
  double pivot_offset_, wheel_radius_;
  effort_controllers::JointPositionController* ctrl_pivot_;
  effort_controllers::JointVelocityController* ctrl_wheel_;
};

class SwerveController : public ChassisBase
{
public:
  SwerveController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

private:
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;
  std::vector<Module> modules_;
};

}  // namespace chassis_controllers
