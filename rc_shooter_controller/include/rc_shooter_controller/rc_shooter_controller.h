//
// Created by jialonglong on 23-3-31.
//
#pragma once
#include "controller_interface/multi_interface_controller.h"
#include "hardware_interface/joint_state_interface.h"
#include "pluginlib/class_list_macros.h"
#include "effort_controllers/joint_velocity_controller.h"
#include "effort_controllers/joint_position_controller.h"
#include "hardware_interface/joint_command_interface.h"
#include <realtime_tools/realtime_publisher.h>
#include "ros/ros.h"
#include "rc_msgs/ShooterCmd.h"

namespace rc_shooter_controller
{
    class Controller
            : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface>
    {
    public:
        Controller()=default;
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh ,ros::NodeHandle& controller_nh) override;
        void update(const ros::Time &time, const ros::Duration &period) override;
        void starting(const ros::Time &time) override;

    private:
        void same(const ros::Time &time,const ros::Duration &period);
        void diff(const ros::Time &time,const ros::Duration &period);
        void moveJoint(const ros::Time &time,const ros::Duration &period);
        void VelCallback(const rc_msgs::ShooterCmdConstPtr& msg);
        //ros_functions
        ros::Subscriber vel_sub_;
        //parameters
        double wheel_radius_{},emission_mass_{},emission_coefficient_{},acceleration_time_{},
                    diff_ratio_{},publish_rate_{},timeout_{},limiting_coefficient_{};
        std::string left_wheel_joint_, right_wheel_joint_;
        rc_msgs::ShooterCmd Shooter_Cmd_;
        realtime_tools::RealtimeBuffer<rc_msgs::ShooterCmd> Shooter_Cmd_Buffer_;
        double final_speed_;
        double left_speed_command_,right_speed_command_;
        bool state_changed_{};
        bool command_received_{};
        effort_controllers::JointVelocityController left_joint_ctrl_,right_joint_ctrl_;

        enum{
            SAME,
            DIFF
        };
        int state_=SAME;


    };
    PLUGINLIB_EXPORT_CLASS(rc_shooter_controller::Controller, controller_interface::ControllerBase)
}//namespace rc_shooter_controller
