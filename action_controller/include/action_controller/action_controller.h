//
// Created by myx on 2022/11/13.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <rc_common/hardware_interface/action_interface.h>
#include <rc_common/tf_rt_broadcaster.h>
#include <realtime_tools/realtime_publisher.h>
#include <rc_msgs/ActionData.h>
#include <rc_msgs/ActionCmd.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <queue>

namespace action_controller
{
class Controller : public controller_interface::MultiInterfaceController<rc_control::ActionInterface>
{
public:
  Controller() = default;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

  bool setActionCmd(rc_msgs::ActionCmd::Request& req, rc_msgs::ActionCmd::Response& resp);

private:
  std::vector<std::string> action_names_{};
  std::vector<rc_control::ActionHandle> action_handles_{};

  typedef std::shared_ptr<realtime_tools::RealtimePublisher<rc_msgs::ActionData>> RtpublisherPtr;
  std::vector<RtpublisherPtr> action_data_pubs_{};

  geometry_msgs::TransformStamped odom2base_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_{};
  rc_common::TfRtBroadcaster tf_broadcaster_{};
  ros::ServiceServer action_cmd_service_{};

  double publish_rate_{};
  bool pub_odom_tf_{};
  bool publish_action_data_{};
  ros::Time last_publish_time_;

  geometry_msgs::Vector3 last_pose_;
};
}  // namespace action_controller
