//
// Created by jialonglong on 23-3-31.
//
#include "rc_shooter_controller/rc_shooter_controller.h"
namespace rc_shooter_controller
{
    bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh ,ros::NodeHandle& controller_nh) {
        if(!controller_nh.getParam("publish_rate", publish_rate_)|| !controller_nh.getParam("timeout", timeout_)) {
            ROS_ERROR("Some chassis params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
            return false;
        }
        if(!controller_nh.getParam("wheel_radius", wheel_radius_)||
        !controller_nh.getParam("emission_mass",emission_mass_)||
        !controller_nh.getParam("emission_coefficient",emission_coefficient_)||
        !controller_nh.getParam("acceleration_time",acceleration_time_)||
        !controller_nh.getParam("diff_ratio",diff_ratio_)||
        !controller_nh.getParam("limiting_coefficient",limiting_coefficient_)) {
            ROS_ERROR("Failed to get parameters for rc_shooter_controller");
            return false;
        }
        vel_sub_=controller_nh.subscribe<rc_msgs::ShooterCmd>("command",1,&Controller::VelCallback, this);
        //get left and right friction wheels
        ros::NodeHandle left_nh =ros::NodeHandle(controller_nh,"left_friction_wheel");
        ros::NodeHandle right_nh =ros::NodeHandle(controller_nh,"right_friction_wheel");
        if (!left_joint_ctrl_.init(robot_hw->get<hardware_interface::EffortJointInterface>(), left_nh)||
        !right_joint_ctrl_.init(robot_hw->get<hardware_interface::EffortJointInterface>(),right_nh)){
            ROS_ERROR("Failed to initialize left and right friction wheels");
            return false;
        }
        return true;
    }

    void Controller::starting(const ros::Time &time) {
        state_= SAME;
        state_changed_=true;
    }

    void Controller::update(const ros::Time &time, const ros::Duration &period) {
        shooter_cmd_ = *shooter_cmd_buffer_.readFromRT();
        if(state_!=shooter_cmd_.mode){
            state_=shooter_cmd_.mode;
            state_changed_=true;
        }
        final_speed_=shooter_cmd_.speed;
        switch (state_) {
            case SAME:
                same(time,period);
                break;
            case DIFF:
                diff(time,period);
                break;
        }
        moveJoint(time,period);
    }

    void Controller::same(const ros::Time &time, const ros::Duration &period) {
        if(state_changed_){
            state_changed_=false;
            ROS_INFO("[Shooter_Controller] SAME MODE");}
        left_speed_command_=final_speed_/wheel_radius_;
        right_speed_command_=final_speed_/wheel_radius_;
        moveJoint(time,period);
    }

    void Controller::diff(const ros::Time &time, const ros::Duration &period) {
        if(state_changed_){
            state_changed_=false;
            ROS_INFO("[Shooter_Controller] DIFF MODE");}
        left_speed_command_=(final_speed_*(1-diff_ratio_))/wheel_radius_;
        right_speed_command_=(final_speed_*(1+diff_ratio_))/wheel_radius_;
        moveJoint(time,period);
    }

    void Controller::moveJoint(const ros::Time &time, const ros::Duration &period) {
        if (!command_received_){
            left_speed_command_=0.0;
            right_speed_command_=0.0;
            ROS_WARN("Command not received yet");
        }
        double emission_force = emission_mass_ * acceleration_time_ / period.toSec();
        double wheel_torque = emission_force * wheel_radius_ * emission_coefficient_*limiting_coefficient_;
        left_speed_command_ = copysign(std::min(std::abs(left_speed_command_), wheel_torque), left_speed_command_);
        right_speed_command_ = copysign(std::min(std::abs(right_speed_command_), wheel_torque), right_speed_command_);

        left_joint_ctrl_.setCommand(-left_speed_command_);
        right_joint_ctrl_.setCommand(right_speed_command_);
        left_joint_ctrl_.update(time,period);
        right_joint_ctrl_.update(time,period);
    }

    void Controller::VelCallback(const rc_msgs::ShooterCmdConstPtr &msg) {
        shooter_cmd_buffer_.writeFromNonRT(*msg);
        if (ros::topic::waitForMessage<rc_msgs::ShooterCmd>("/controllers/shooter_controller/command", ros::Duration(timeout_))) {
            command_received_ = true;
        } else {
            command_received_ = false;
        }
    }

}//namespace rc_shooter_controller