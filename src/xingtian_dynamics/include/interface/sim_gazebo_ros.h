/**********************************************************************
 Copyright (c) 2024, CIEM——ZYZ. All rights reserved.
***********************************************************************/

// #ifdef COMPILE_WITH_ROS
#ifndef COMPILE_WITH_ROS
#define COMPILE_WITH_ROS    //自己添加的定义
#ifndef SIM_GAZEBO_ROS_H
#define SIM_GAZEBO_ROS_H

#include "ros/ros.h"
#include "interface/IOInterface.h"
#include "xingtian_msgs/LowCmd.h"
#include "xingtian_msgs/LowState.h"
#include "xingtian_msgs/MotorCmd.h"
#include "xingtian_msgs/MotorState.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include "KeyBoard.h"
#include "real_robot/rt_rc_interface.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>


#include "control/CtrlComponents.h"
// File: IOROS.h
// template<typename T>
// class DesiredStateCommand; // 前向声明
template <typename T>
class IOROS : 
public IOInterface<T>{
public:
    IOROS();
    ~IOROS();
    void sendRecv(const LowlevelCmd<T> *cmd, LowlevelState<T> *state);
    // void setDesiredStateCommand(DesiredStateCommand<T>* cmd) {
    //     _desiredStateCommand = cmd; // 延迟初始化
    // }
    // KeyBoard<T>* getKeyboard() override{ return _keyboard; }
    rc_control_settings* getRCControl()override {return _rc_control;}
    T result_tau[12];
private:
    void sendCmd(const LowlevelCmd<T> *cmd);
    void recvState(LowlevelState<T> *state);
    ros::NodeHandle _nm;
    ros::Subscriber _servo_sub[14], _imu_sub, _joint_states;
    ros::Publisher _servo_pub[13];
    // message_filters::Subscriber<sensor_msgs::Image> _image_sub_;     //图像订阅话题
    // message_filters::Subscriber<sensor_msgs::CameraInfo> _info_sub_;
    // message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo>* sync_;
    xingtian_msgs::LowCmd _lowCmd;
    xingtian_msgs::LowState _lowState;
    std::string _robot_name;
    // DesiredStateCommand<T> *_desiredStateCommand;
    //repeated functions for multi-thread
    void initRecv();
    void initSend();
    T degreesToRadians(T degrees);
    T radiansToDegrees(T degrees);
    //Callback functions for ROS
    void imuCallback(const sensor_msgs::Imu & msg);
    void jointStateCallback(const sensor_msgs::JointState & msg);
    void RRhipCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void RRkneeCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void RRwheelCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    void LRhipCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void LRkneeCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void LRwheelCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    void LFhipCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void LFkneeCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void LFwheelCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    void RFhipCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void RFkneeCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void RFwheelCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void Model_stateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    // void FL_foot_force(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    // void RL_foot_force(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    // void RR_foot_force(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    // void FR_foot_force(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void Laser_callback(const std_msgs::Float64::ConstPtr& msg);
    void Camera_callback(const sensor_msgs::ImageConstPtr& msg);


    rc_control_settings* _rc_control;
};

#endif  // SIM_GAZEBO_ROS_H
#endif  // COMPILE_WITH_ROS