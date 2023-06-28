//
// Created by myx on 2022/11/13.
//

#include "action_controller/action_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <rc_common/ros_utilities.h>
#include <rc_common/ori_tool.h>

namespace action_controller
{
    bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
    {
        XmlRpc::XmlRpcValue xml_rpc_value;
        controller_nh.getParam("action_names", xml_rpc_value);
        ROS_ASSERT(xml_rpc_value.getType() == XmlRpc::XmlRpcValue::TypeArray);
        if (xml_rpc_value.size() <= 0)
        {
            ROS_ERROR_STREAM("Action_names params doesn't given (namespace: %s)" << controller_nh.getNamespace());
            return false;
        }
        for (int i = 0; i < xml_rpc_value.size(); i++)
        {
            ROS_ASSERT(xml_rpc_value[i].getType() == XmlRpc::XmlRpcValue::TypeString);
            action_names_.push_back(xml_rpc_value[i]);
        }
        if (!controller_nh.getParam("pub_rate", publish_rate_) || !controller_nh.getParam("pub_odom_tf", pub_odom_tf_) ||
            !controller_nh.getParam("pub_action_data", publish_action_data_))
        {
            ROS_ERROR_STREAM("Some actions params doesn't given：" << controller_nh.getNamespace());
            return false;
        }
        XmlRpc::XmlRpcValue twist_cov_list;
        controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
        ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(twist_cov_list.size() == 6);
        for (int i = 0; i < twist_cov_list.size(); ++i)
            ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

        for (int i = 0; i < action_names_.size(); i++)
        {
            rc_control::ActionHandle action_handle = robot_hw->get<rc_control::ActionInterface>()->getHandle(action_names_[i]);
            ROS_INFO_STREAM("get action handle success .");
            action_handles_.push_back(action_handle);
            if (publish_action_data_)
            {
                std::shared_ptr<realtime_tools::RealtimePublisher<rc_msgs::ActionData>> action_data_pub;
                action_data_pub.reset(
                        new realtime_tools::RealtimePublisher<rc_msgs::ActionData>(root_nh, action_names_[i] + "_odom", 100));
                action_data_pubs_.push_back(action_data_pub);
            }
        }

        last_pose_.x = action_handles_[0].getPoseX();
        last_pose_.y = action_handles_[0].getPoseY();
        last_pose_.z = action_handles_[0].getYawAngle();

        if (pub_odom_tf_)
        {
            odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(root_nh, "odom", 100));
            odom_pub_->msg_.header.frame_id = "odom";
            odom_pub_->msg_.child_frame_id = "base_link";
            odom_pub_->msg_.twist.covariance = { static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0., 0.,
                                                 static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0., 0., 0.,
                                                 static_cast<double>(twist_cov_list[2]), 0., 0., 0., 0., 0., 0.,
                                                 static_cast<double>(twist_cov_list[3]), 0., 0., 0., 0., 0., 0.,
                                                 static_cast<double>(twist_cov_list[4]), 0., 0., 0., 0., 0., 0.,
                                                 static_cast<double>(twist_cov_list[5]) };
            odom2base_.header.frame_id = "odom";
            odom2base_.header.stamp = ros::Time::now();
            odom2base_.child_frame_id = "base_link";
            odom2base_.transform.rotation.w = 1;
            tf_broadcaster_.init(root_nh);
            tf_broadcaster_.sendTransform(odom2base_);
        }
        action_cmd_service_ = controller_nh.advertiseService("action_cmd", &Controller::setActionCmd, this);
        ibus_listener_=controller_nh.subscribe<rc_msgs::IbusData>("ibus_data",1,&Controller::IbusData,this);
        last_publish_time_ = ros::Time::now();
        return true;
    }

    void Controller::update(const ros::Time& time, const ros::Duration& period)
    {
        ibus_data=*cmd_buffer_ibus.readFromNonRT();
        static bool enable_update_odom_=false;
        ros::Time last_period_time;
        nav_msgs::Odometry last_nav_msgs;
        if (publish_rate_ > 0.0 && time - last_publish_time_ >= ros::Duration(1 / publish_rate_))
        {
            double getPoseX = action_handles_[0].getPoseX();
            double getPoseY = action_handles_[0].getPoseY();
            double getPitchAngle = action_handles_[0].getPitchAngle();
            double getYawAngle = action_handles_[0].getYawAngle();
            if (pub_odom_tf_)
            {
                // publish tf
                odom2base_.header.stamp = time;
                odom2base_.transform.translation.x = getPoseX;
                odom2base_.transform.translation.y = getPoseY;
                odom2base_.transform.translation.z = 0;
                odom2base_.transform.rotation = rpyToQuat(action_handles_[0].getRollAngle(), getPitchAngle, getYawAngle);
                tf_broadcaster_.sendTransform(odom2base_);
                // publish odom
                if (odom_pub_->trylock())
                {
                    //normal state
                    odom_pub_->msg_.header.stamp = time;
                    odom_pub_->msg_.pose.pose.orientation =
                            rpyToQuat(action_handles_[0].getRollAngle(), getPitchAngle, getYawAngle);
                    odom_pub_->msg_.twist.twist.linear.x = (getPoseX - last_pose_.x) / period.toSec();
                    odom_pub_->msg_.twist.twist.linear.y = (getPoseY - last_pose_.y) / period.toSec();
                    odom_pub_->msg_.twist.twist.angular.z = (getYawAngle - last_pose_.z) / period.toSec();
                    odom_pub_->unlockAndPublish();

                    //listen to ibus_data to update position
                    if(ibus_data.sw_c==2)
                    {
                        enable_update_odom_=true;
                    }
                    if(enable_update_odom_) {
                        last_period_time=ros::Time::now();
                        odom_pub_->msg_.pose.pose.position.x =
                                -0.25 + (getPoseX-last_nav_msgs.pose.pose.position.x);
                        odom_pub_->msg_.pose.pose.position.y =
                                -1.75 + (getPoseY-last_nav_msgs.pose.pose.position.y);
                    }
                    else
                    {
                        odom_pub_->msg_.pose.pose.position.x = getPoseX;
                        odom_pub_->msg_.pose.pose.position.y = getPoseY;
                        last_nav_msgs.pose.pose.position.x=odom_pub_->msg_.pose.pose.position.x;
                        last_nav_msgs.pose.pose.position.y=odom_pub_->msg_.pose.pose.position.y;

                    }
                }
            }
            if (publish_action_data_)
            {
                // publish actions state
                for (int i = 0; i < action_handles_.size(); ++i)
                {
                    if (action_data_pubs_[i]->trylock())
                    {
                        action_data_pubs_[i]->msg_.header.stamp = time;
                        action_data_pubs_[i]->msg_.pose_x = action_handles_[i].getPoseX();
                        action_data_pubs_[i]->msg_.pose_y = action_handles_[i].getPoseY();
                        action_data_pubs_[i]->msg_.roll_angle = action_handles_[i].getRollAngle();
                        action_data_pubs_[i]->msg_.pitch_angle = action_handles_[i].getPitchAngle();
                        action_data_pubs_[i]->msg_.yaw_angle = action_handles_[i].getYawAngle();
                        action_data_pubs_[i]->msg_.yaw_acc = action_handles_[i].getYawAcc();
                        action_data_pubs_[i]->unlockAndPublish();
                    }
                }
            }
            last_pose_.x = getPoseX;
            last_pose_.y = getPoseY;
            last_pose_.z = getYawAngle;
            last_publish_time_ = ros::Time::now();
        }
    }

    bool Controller::setActionCmd(rc_msgs::ActionCmd::Request& req, rc_msgs::ActionCmd::Response& resp)
    {
        for (auto item : action_handles_)
        {
            if (item.getName().compare(req.action_name) == 0)
            {
                if (req.calibration_enable)
                {
                    ROS_WARN_STREAM("Don't suggest calibration action by this method, so this commend is not execute.");
                    // item.setCalibration();
                }
                if (req.reset_enable)
                {
                    item.resetAction();
                }
                if (req.updateX_enable)
                {
                    item.updatePoseX(req.updateX_data);
                }
                if (req.updateY_enable)
                {
                    item.updatePoseY(req.updateY_data);
                }
                if (req.updateY_enable)
                {
                    item.updatePoseYaw(req.updateYaw_data);
                }
                resp.is_success = true;
            }
        }
        return true;
    }

    void Controller::IbusData(const rc_msgs::IbusDataConstPtr &msg){
        cmd_buffer_ibus.writeFromNonRT(*msg);
    }

}  // namespace action_controller

PLUGINLIB_EXPORT_CLASS(action_controller::Controller, controller_interface::ControllerBase)