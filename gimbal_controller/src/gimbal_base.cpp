//
// Created by jialonglong on 2023/3/15.
//
// ref:https://github.com/rm-controls
#include "gimbal_controller/gimbal_base.h"
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace gimbal_controller {
    bool Controller::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
        if (!controller_nh.getParam("time_out", time_out_)||!controller_nh.getParam("publish_rate", publish_rate_)) {
            ROS_ERROR("time_out or publish_rate is not set.");
            return false;
        }
        if(time_out_>=1/publish_rate_)
        {
            ROS_ERROR("Please reset time_out and publish_rate. Time_out should be less than 1/publish_rate.");
            return false;
        }
        // create tf buffer and listener
        tf_buffer_.reset(new tf2_ros::Buffer());
        tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
        // get yaw joint and pitch joint or roll joint
        ros::NodeHandle nh_yaw = ros::NodeHandle(controller_nh, "yaw");
        ros::NodeHandle nh_pitch = ros::NodeHandle(controller_nh, "pitch");
        hardware_interface::EffortJointInterface *effort_joint_interface =
                robot_hw->get<hardware_interface::EffortJointInterface>();
        // Two degrees of freedom
        if (!ctrl_yaw_.init(effort_joint_interface, nh_yaw) || !ctrl_pitch_.init(effort_joint_interface, nh_pitch))
            return false;
        // Coordinate transformation of the head itself
        gimbal_des_frame_id_ = ctrl_pitch_.joint_urdf_->child_link_name+"des";
        odom2gimbal_des_.header.frame_id = "base_link";
        odom2gimbal_des_.child_frame_id = gimbal_des_frame_id_;
        odom2gimbal_des_.transform.rotation.w = 1.;
        odom2pitch_.header.frame_id = "base_link";
        odom2pitch_.child_frame_id = ctrl_pitch_.joint_urdf_->child_link_name;
        odom2pitch_.transform.rotation.w = 1.;
        odom2base_.header.frame_id = "base_link";
        odom2base_.child_frame_id = ctrl_yaw_.joint_urdf_->parent_link_name;
        odom2base_.transform.rotation.w = 1.;

        cmd_gimbal_sub_ = controller_nh.subscribe<rc_msgs::GimbalCmd>("command", 1, &Controller::commandCB, this);
        publish_rate_ = getParam(controller_nh, "publish_rate", 100.);
        error_pub_.reset(new realtime_tools::RealtimePublisher<rc_msgs::GimbalDesError>(controller_nh, "error", 100));
        return true;
    }

    void Controller::starting(const ros::Time & /*unused*/) {
        state_ = RATE;
        state_changed_ = true;
    }

    void Controller::update(const ros::Time &time, const ros::Duration &period) {
        cmd_gimbal_ = *cmd_rt_buffer_.readFromRT();
        try {
            odom2pitch_ = tf_buffer_->lookupTransform("base_link", ctrl_pitch_.joint_urdf_->child_link_name, ros::Time(0),
                                                      ros::Duration(time_out_));
            odom2base_ = tf_buffer_->lookupTransform("base_link", ctrl_yaw_.joint_urdf_->parent_link_name, ros::Time(0),
                                                     ros::Duration(time_out_));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }
        if (state_ != cmd_gimbal_.mode) {
            state_ = cmd_gimbal_.mode;
            state_changed_ = true;
        }
        switch (state_) {
            case RATE:
                rate(time, period);
                break;
            case NORMAL:
                normal(time);
                break;
        }
        moveJoint(time, period);

    }

    void Controller::setDes(const ros::Time &time, double yaw_des, double pitch_des) {
        tf2::Quaternion odom2base, odom2gimbal_des;
        tf2::Quaternion base2gimbal_des;
        tf2::fromMsg(odom2base_.transform.rotation, odom2base);
        odom2gimbal_des.setRPY(0, pitch_des, yaw_des);
        base2gimbal_des = odom2base.inverse() * odom2gimbal_des;
        double roll_temp, base2gimbal_current_des_pitch, base2gimbal_current_des_yaw;
        quatToRPY(toMsg(base2gimbal_des), roll_temp, base2gimbal_current_des_pitch, base2gimbal_current_des_yaw);
        double pitch_real_des, yaw_real_des;

        if (!setDesIntoLimit(pitch_real_des, pitch_des, base2gimbal_current_des_pitch, ctrl_pitch_.joint_urdf_))
        {
            double yaw_temp;
            tf2::Quaternion base2new_des;
            double upper_limit, lower_limit;
            upper_limit = ctrl_pitch_.joint_urdf_->limits ? ctrl_pitch_.joint_urdf_->limits->upper : 1e16;
            lower_limit = ctrl_pitch_.joint_urdf_->limits ? ctrl_pitch_.joint_urdf_->limits->lower : -1e16;
            base2new_des.setRPY(0,
                                std::abs(angles::shortest_angular_distance(base2gimbal_current_des_pitch, upper_limit)) <
                                std::abs(angles::shortest_angular_distance(base2gimbal_current_des_pitch, lower_limit)) ?
                                upper_limit :
                                lower_limit,
                                base2gimbal_current_des_yaw);
            quatToRPY(toMsg(odom2base * base2new_des), roll_temp, pitch_real_des, yaw_temp);
        }

        if (!setDesIntoLimit(yaw_real_des, yaw_des, base2gimbal_current_des_yaw, ctrl_yaw_.joint_urdf_))
        {
            double pitch_temp;
            tf2::Quaternion base2new_des;
            double upper_limit, lower_limit;
            upper_limit = ctrl_yaw_.joint_urdf_->limits ? ctrl_yaw_.joint_urdf_->limits->upper : 1e16;
            lower_limit = ctrl_yaw_.joint_urdf_->limits ? ctrl_yaw_.joint_urdf_->limits->lower : -1e16;
            base2new_des.setRPY(0, base2gimbal_current_des_pitch,
                                std::abs(angles::shortest_angular_distance(base2gimbal_current_des_yaw, upper_limit)) <
                                std::abs(angles::shortest_angular_distance(base2gimbal_current_des_yaw, lower_limit)) ?
                                upper_limit :
                                lower_limit);
            quatToRPY(toMsg(odom2base * base2new_des), roll_temp, pitch_temp, yaw_real_des);
        }

        odom2gimbal_des_.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0., pitch_real_des, yaw_real_des);
        odom2gimbal_des_.header.stamp = time;
        tf_buffer_->setTransform(odom2gimbal_des_, "gimbal_controller", false);
    }

    void Controller::rate(const ros::Time &time, const ros::Duration &period) {
        if (state_changed_) {  // on enter
            state_changed_ = false;
            ROS_INFO("[Gimbal] Enter RATE");
            odom2gimbal_des_.transform.rotation = odom2pitch_.transform.rotation;
            odom2gimbal_des_.header.stamp = time;
            tf_buffer_->setTransform(odom2gimbal_des_, "gimbal_controller", false);
        } else {
            double roll{}, pitch{}, yaw{};
            quatToRPY(odom2gimbal_des_.transform.rotation, roll, pitch, yaw);
            setDes(time, yaw + period.toSec() * cmd_gimbal_.rate_yaw, pitch + period.toSec() * cmd_gimbal_.rate_pitch);
        }
    }

    void Controller::direct(const ros::Time &time) {
        if (state_changed_) {
            state_changed_ = false;
            ROS_INFO("[Gimbal] Enter DIRECT");
        }
        geometry_msgs::Point aim_point_odom = cmd_gimbal_.target_pos.point;
        try {
            if (!cmd_gimbal_.target_pos.header.frame_id.empty())
                tf2::doTransform(aim_point_odom, aim_point_odom,
                                 tf_buffer_->lookupTransform("base_link", cmd_gimbal_.target_pos.header.frame_id,
                                                             cmd_gimbal_.target_pos.header.stamp, ros::Duration(time_out_)));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
        double yaw = std::atan2(aim_point_odom.y - odom2pitch_.transform.translation.y,
                                aim_point_odom.x - odom2pitch_.transform.translation.x);
        double pitch = -std::atan2(aim_point_odom.z - odom2pitch_.transform.translation.z,
                                   std::sqrt(std::pow(aim_point_odom.x - odom2pitch_.transform.translation.x, 2) +
                                             std::pow(aim_point_odom.y - odom2pitch_.transform.translation.y, 2)));
        setDes(time, yaw, pitch);
    }

    void Controller::normal(const ros::Time &time) {
        if (state_changed_) {
            state_changed_ = false;
            ROS_INFO("[Gimbal] Enter NORMAL");
        }
        double yaw = cmd_gimbal_.yaw_target_pos;
        double pitch = cmd_gimbal_.pitch_target_pos;
        setDes(time, yaw, pitch);
    }

    bool Controller::setDesIntoLimit(double &real_des, double current_des, double base2gimbal_current_des,
                                     const urdf::JointConstSharedPtr &joint_urdf) {
        double upper_limit, lower_limit;
        upper_limit = joint_urdf->limits ? joint_urdf->limits->upper : 1e16;
        lower_limit = joint_urdf->limits ? joint_urdf->limits->lower : -1e16;
        if ((base2gimbal_current_des <= upper_limit && base2gimbal_current_des >= lower_limit) ||
            (angles::two_pi_complement(base2gimbal_current_des) <= upper_limit &&
             angles::two_pi_complement(base2gimbal_current_des) >= lower_limit))
            real_des = current_des;
        else
            return false;
        return true;
    }

    void Controller::moveJoint(const ros::Time &time, const ros::Duration &period) {
        geometry_msgs::Vector3 gyro, angular_vel_pitch, angular_vel_yaw;
            angular_vel_yaw.z = ctrl_yaw_.joint_.getVelocity();
            angular_vel_pitch.y = ctrl_pitch_.joint_.getVelocity();
        geometry_msgs::TransformStamped base_frame2des;
        base_frame2des =
                tf_buffer_->lookupTransform(ctrl_yaw_.joint_urdf_->parent_link_name, gimbal_des_frame_id_, ros::Time(0),
                                            ros::Duration(time_out_));
        double roll_des, pitch_des, yaw_des;  // desired position
        quatToRPY(base_frame2des.transform.rotation, roll_des, pitch_des, yaw_des);

        double yaw_vel_des = 0., pitch_vel_des = 0.;
        if (state_ == RATE) {
            yaw_vel_des = cmd_gimbal_.rate_yaw;
            pitch_vel_des = cmd_gimbal_.rate_pitch;
        }
        if(state_ == NORMAL){
            yaw_des=cmd_gimbal_.yaw_target_pos;
            pitch_des=cmd_gimbal_.pitch_target_pos;
            yaw_vel_des = cmd_gimbal_.rate_yaw;
            pitch_vel_des = cmd_gimbal_.rate_pitch;
        }

        ctrl_yaw_.setCommand(yaw_des, yaw_vel_des + ctrl_yaw_.joint_.getVelocity() - angular_vel_yaw.z);
        ctrl_pitch_.setCommand(pitch_des, pitch_vel_des + ctrl_pitch_.joint_.getVelocity() - angular_vel_pitch.y);
        ctrl_yaw_.update(time, period);
        ctrl_pitch_.update(time, period);
        ctrl_yaw_.joint_.setCommand(ctrl_yaw_.joint_.getCommand());
        ctrl_pitch_.joint_.setCommand(ctrl_pitch_.joint_.getCommand());
    }

    void Controller::commandCB(const rc_msgs::GimbalCmdConstPtr &msg) {
        cmd_rt_buffer_.writeFromNonRT(*msg);
    }


}  // namespace gimbal_controller

PLUGINLIB_EXPORT_CLASS(gimbal_controller::Controller, controller_interface::ControllerBase)
