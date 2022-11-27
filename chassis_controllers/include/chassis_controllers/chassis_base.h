//
// Created by myx on 2022/11/15.
//
// ref:https://github.com/rm-controls
#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <rc_common/filters/filters.h>
#include <rc_msgs/ChassisCmd.h>
#include <geometry_msgs/TwistStamped.h>

namespace chassis_controllers
{
struct Command
{
  geometry_msgs::Twist cmd_vel_;
  rc_msgs::ChassisCmd cmd_chassis_;
  ros::Time stamp_;
};

class ChassisBase : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface>
{
public:
  ChassisBase() = default;
  /** @brief Get and check params for covariances. Setup odometry realtime publisher + odom message constant fields.
   * init odom tf.
   *
   * @param robot_hw The robot hardware abstraction.
   * @param root_nh A NodeHandle in the root of the controller manager namespace. This is where the ROS interfaces are
   * setup (publishers, subscribers, services).
   * @param controller_nh A NodeHandle in the namespace of the controller. This is where the controller-specific
   * configuration resides.
   * @return True if initialization was successful and the controller
   * is ready to be started.
   */
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  /** @brief Receive real_time command from manual. Execute different action according to current mode. Set
   * necessary params of chassis. Execute power limit.
   *
   * Receive real_time command from manual and check whether it is normally, if can not receive command from manual
   * for a while, chassis's velocity will be set zero to avoid out of control. Execute different action according
   * to current mode such as RAW, FOLLOW, GYRO, TWIST.(Their specific usage will be explain in the next). UpdateOdom,Set
   * necessary params such as Acc and vel_tfed. Execute moving action and powerlimit.
   *
   * @param time The current time.
   * @param period The time passed since the last call to update.
   */
  void update(const ros::Time& time, const ros::Duration& period) override;

protected:
  virtual void moveJoint(const ros::Time& time, const ros::Duration& period) = 0;

  /** @brief Set chassis velocity to zero.
   */
  void recovery();
  /** @brief Transform tf velocity to base link frame.
   *
   * @param from The father frame.
   */
  void tfVelToBase(const std::string& from);

  /** @brief Write current command from rc_msgs::ChassisCmd.
   *
   * @param msg This message contains various state parameter settings for basic chassis control
   */
  void cmdChassisCallback(const rc_msgs::ChassisCmdConstPtr& msg);
  /** @brief Write current command from  geometry_msgs::Twist.
   *
   * @param msg This expresses velocity in free space broken into its linear and angular parts.
   */
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

  hardware_interface::EffortJointInterface* effort_joint_interface_{};
  std::vector<hardware_interface::JointHandle> joint_handles_{};

  double wheel_base_{}, wheel_track_{}, wheel_radius_{}, publish_rate_{}, twist_angular_{}, timeout_{}, effort_coeff_{},
      velocity_coeff_{}, power_offset_{};

  RampFilter<double>*ramp_x_{}, *ramp_y_{}, *ramp_w_{};
  std::string command_source_frame_{};

  geometry_msgs::Vector3 vel_cmd_{};  // x, y
  control_toolbox::Pid pid_follow_;

  ros::Subscriber cmd_chassis_sub_;
  ros::Subscriber cmd_vel_sub_;
  Command cmd_struct_;
  realtime_tools::RealtimeBuffer<Command> cmd_rt_buffer_;
};

}  // namespace chassis_controllers