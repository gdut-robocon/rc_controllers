//
// Created by myx on 2022/11/15.
//
// ref:https://github.com/rm-controls
#include "chassis_controllers/chassis_base.h"
#include <rc_common/ros_utilities.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace chassis_controllers
{

bool ChassisBase::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  if (!controller_nh.getParam("publish_rate", publish_rate_) || !controller_nh.getParam("timeout", timeout_))
  {
    ROS_ERROR("Some chassis params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  // https://en.wikipedia.org/wiki/Wheelbase
  wheel_radius_ = getParam(controller_nh, "wheel_radius", 0.02);
  wheel_track_ = getParam(controller_nh, "wheel_track", 0.410);
  wheel_base_ = getParam(controller_nh, "wheel_base", 0.320);

  effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();

  ramp_x_ = new RampFilter<double>(0, 0.001);
  ramp_y_ = new RampFilter<double>(0, 0.001);
  ramp_w_ = new RampFilter<double>(0, 0.001);

  cmd_chassis_sub_ = controller_nh.subscribe<rc_msgs::ChassisCmd>("command", 1, &ChassisBase::cmdChassisCallback, this);
  cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &ChassisBase::cmdVelCallback, this);

  return true;
}

void ChassisBase::update(const ros::Time& time, const ros::Duration& period)
{
  rc_msgs::ChassisCmd cmd_chassis = cmd_rt_buffer_.readFromRT()->cmd_chassis_;
  geometry_msgs::Twist cmd_vel = cmd_rt_buffer_.readFromRT()->cmd_vel_;

  if ((time - cmd_rt_buffer_.readFromRT()->stamp_).toSec() > timeout_)
  {
    vel_cmd_.x = 0.;
    vel_cmd_.y = 0.;
    vel_cmd_.z = 0.;
  }
  else
  {
    ramp_x_->setAcc(cmd_chassis.accel.linear.x);
    ramp_y_->setAcc(cmd_chassis.accel.linear.y);
    ramp_x_->input(cmd_vel.linear.x);
    ramp_y_->input(cmd_vel.linear.y);
    vel_cmd_.x = ramp_x_->output();
    vel_cmd_.y = ramp_y_->output();
    vel_cmd_.z = cmd_vel.angular.z;
  }

  if (cmd_rt_buffer_.readFromRT()->cmd_chassis_.command_source_frame.empty())
    command_source_frame_ = "base_link";
  else
    command_source_frame_ = cmd_rt_buffer_.readFromRT()->cmd_chassis_.command_source_frame;
  ramp_w_->setAcc(cmd_chassis.accel.angular.z);
  ramp_w_->input(vel_cmd_.z);
  vel_cmd_.z = ramp_w_->output();
  moveJoint(time, period);
}

void ChassisBase::recovery()
{
  ramp_x_->clear(vel_cmd_.x);
  ramp_y_->clear(vel_cmd_.y);
  ramp_w_->clear(vel_cmd_.z);
}

void ChassisBase::tfVelToBase(const std::string& from)
{
  try
  {
    tf2::doTransform(vel_cmd_, vel_cmd_,
                     tf2_ros::Buffer(ros::Duration(5)).lookupTransform("base_link", from, ros::Time(0)));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

void ChassisBase::cmdChassisCallback(const rc_msgs::ChassisCmdConstPtr& msg)
{
  cmd_struct_.cmd_chassis_ = *msg;
  cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
}

void ChassisBase::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmd_struct_.cmd_vel_ = *msg;
  cmd_struct_.stamp_ = ros::Time::now();
  cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
}
}  // namespace chassis_controllers