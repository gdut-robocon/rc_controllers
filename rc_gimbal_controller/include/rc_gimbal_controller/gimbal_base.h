//
// Created by jialonglong on 23/3/15
//
//ref:https://github.com/rm-controls
#pragma once

#include <effort_controllers/joint_position_controller.h>
#include <controller_manager/controller_manager.h>
#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <rc_common/hardware_interface/robot_state_interface.h>
#include <rc_common/ros_utilities.h>
#include <rc_common/ori_tool.h>
#include <rc_common/filters/filters.h>
#include <tf2_eigen/tf2_eigen.h>
#include <dynamic_reconfigure/server.h>
#include <rc_msgs/GimbalCmd.h>
#include <rc_msgs/GimbalDesError.h>
#include <Eigen/Eigen>
#include <boost/scoped_ptr.hpp>

namespace rc_gimbal_controller {
    class Controller : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface> {
    public:
        Controller() = default;

        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        void setDes(const ros::Time &time, double yaw_des, double pitch_des);

        void starting(const ros::Time &time) override;

    private:
        // Functions
        void direct(const ros::Time &time);

        void normal(const ros::Time &time);

        void rate(const ros::Time &time, const ros::Duration &period);

        bool setDesIntoLimit(double &real_des, double current_des, double base2gimbal_current_des,
                             const urdf::JointConstSharedPtr &joint_urdf);

        void moveJoint(const ros::Time &time, const ros::Duration &period);

        void commandCB(const rc_msgs::GimbalCmdConstPtr &msg);


        std::vector<hardware_interface::JointHandle> joint_handles_;
        effort_controllers::JointPositionController ctrl_yaw_, ctrl_pitch_;
        rc_control::RobotStateHandle robot_state_handle_;
        tf2_ros::BufferInterface *tf_buffer_handle_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::string gimbal_des_frame_id_{}, imu_name_{};

        // Transform
        geometry_msgs::TransformStamped odom2gimbal_des_, odom2pitch_, odom2base_, odom2roll_, last_odom2base_;

        std::shared_ptr<realtime_tools::RealtimePublisher<rc_msgs::GimbalDesError>> error_pub_;

        double publish_rate_{},time_out_{};
        bool state_changed_{};

        // rc_msgs
        rc_msgs::GimbalCmd cmd_gimbal_;

        // ROS interface
        realtime_tools::RealtimeBuffer<rc_msgs::GimbalCmd> cmd_rt_buffer_;
        ros::Subscriber cmd_gimbal_sub_;

        enum {
            RATE,
            NORMAL
        };
        int state_ = RATE;
    };

}  // namespace rc_gimbal_controller
