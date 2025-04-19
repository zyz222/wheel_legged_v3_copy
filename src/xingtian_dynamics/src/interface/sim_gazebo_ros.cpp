/**********************************************************************
 Copyright (c) 2024, CIEM-ZYZ. All rights reserved.
 //所有转轴Y轴向右，遵循右手定则
 //待添加车轮与地面的接触力大小，有话题，待实现
***********************************************************************/

#include "interface/sim_gazebo_ros.h"
#include "interface/KeyBoard.h"
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <cmath>
#include "real_robot/rt_rc_interface.h"
// template <typename T>
void RosShutDown(int sig)
{
	ROS_INFO("ROS interface shutting down!");
	ros::shutdown();
}
// 右边是轴的正方向，左边是负方向
template <typename T>
IOROS<T>::IOROS():IOInterface<T>(){
    std::cout << "The control interface for ROS Gazebo simulation" << std::endl;
    // ros::param::get("/robot_name", _robot_name);
    _robot_name = "xingtian";
    std::cout << "robot_name: " << _robot_name << std::endl;
    
    // start subscriber
    initRecv();
    ros::AsyncSpinner subSpinner(1); // one threads  用一个线程来处理回调函数
    subSpinner.start();
    usleep(300000);     //wait for subscribers start 休眠0.3秒
    // initialize publisher
    initSend();   

    signal(SIGINT, RosShutDown);    // ctrl+c to exit
    // std::cout << "崩溃了: "<< std::endl;
    this->_keyboard = new KeyBoard<T>();      
    //这里应该传入参数 输入期望的状态指令
    _rc_control = new rc_control_settings();
    // if (_desiredStateCommand == nullptr) {
    //     // 处理内存分配失败
    //     std::cout << "崩溃了: "<< std::endl;
    // }

    
}
template <typename T>
IOROS<T>::~IOROS(){
    delete this->_keyboard;
    delete _rc_control;
    // delete _desiredStateCommand;
    ros::shutdown();
}
template <typename T>
T IOROS<T>:: degreesToRadians(T degrees) 
{
    return degrees * (M_PI / 180.0);
}
template <typename T>
T IOROS<T>::radiansToDegrees(T radians)
{
    return radians * (180.0 / M_PI);
}
template <typename T>
void IOROS<T>::sendRecv(const LowlevelCmd<T> *cmd, LowlevelState<T> *state)
{
    sendCmd(cmd);
    recvState(state);

    state->userCmd = this->_keyboard->getUserCmd();
    state->userValue = this->_keyboard->getUserValue();


}
// 向gazebo发送数据
template <typename T>
void IOROS<T>::sendCmd(const LowlevelCmd<T> *lowCmd){
    
    for(int i(0); i < 12; ++i)
    {
        // if(i<6)
        _lowCmd.motorCmd[i].mode = lowCmd->motorCmd[i].mode;
        _lowCmd.motorCmd[i].q = lowCmd->motorCmd[i].q;
        // _lowCmd.motorCmd[i].dq = IOROS::degreesToRadians(lowCmd->motorCmd[i].dq);
        _lowCmd.motorCmd[i].dq = lowCmd->motorCmd[i].dq;
        _lowCmd.motorCmd[i].tau = lowCmd->motorCmd[i].tau;    //前馈力矩
        _lowCmd.motorCmd[i].Kd = lowCmd->motorCmd[i].Kd;
        _lowCmd.motorCmd[i].Kp = lowCmd->motorCmd[i].Kp;
        // else{

        // }
        // ROS_WARN("_lowCmd.motorCmd[%d]:%f",i,lowCmd->motorCmd[i].tau);
        // ROS_WARN("kp[%d]:%f",i,lowCmd->motorCmd[i].Kp);
        // ROS_WARN("kd[%d]:%f",i,lowCmd->motorCmd[i].Kd);
    }
    T alpha = 0.9; // 调整速率，值越小变化越慢
    // 进行最终的输出
    for(int j(0); j < 12; ++j)
    {
       
        T target_tau = _lowCmd.motorCmd[j].tau + \
        _lowCmd.motorCmd[j].Kd * (_lowCmd.motorCmd[j].dq - _lowState.motorState[j].dq) + \
         _lowCmd.motorCmd[j].Kp * (_lowCmd.motorCmd[j].q - _lowState.motorState[j].q);

        result_tau[j] = (1 - alpha) * result_tau[j] + alpha * target_tau;
        
    }
    //LF LR RR RF
    for(int m(0); m < 12; ++m)
    {
        std_msgs::Float64 msg;
        msg.data = result_tau[m];
       
        const double MAX_VALUE = 100.0;
        const double MIN_VALUE = -100.0;
        if (msg.data > MAX_VALUE)
        {msg.data = MAX_VALUE;}
        else if(msg.data < MIN_VALUE)
        {msg.data = MIN_VALUE;}
        // msg.data = 0.0;
        _servo_pub[m].publish(msg);

        // ROS_INFO("_servo_pub[%d]:%f",m,msg.data);
    }
    ros::spinOnce();   //处理回调消息
}
template <typename T>
void IOROS<T>::recvState(LowlevelState<T> *state){
    for(int i(0); i < 12; ++i){
        state->motorState[i].q = _lowState.motorState[i].q;
        // state->motorState[i].dq = IOROS::radiansToDegrees(_lowState.motorState[i].dq);
        state->motorState[i].dq = _lowState.motorState[i].dq;
        state->motorState[i].ddq = _lowState.motorState[i].ddq;
        state->motorState[i].tauEst = _lowState.motorState[i].tauEst;
        state->xingtian_state.wheel_force[i] = _lowState.xingtian_state.wheel_force[i];
    }
    for(int i(0); i < 3; ++i){
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
        state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
        state->xingtian_state.position[i] = _lowState.xingtian_state.position[i];
        state->xingtian_state.vBody[i] = _lowState.xingtian_state.vBody[i];
        state->xingtian_state.omegaBody[i] = _lowState.xingtian_state.omegaBody[i];
        state->xingtian_state.orientation[i] = _lowState.xingtian_state.orientation[i];
        state->xingtian_state.acceleration[i] = _lowState.xingtian_state.acceleration[i];
    }
    state->xingtian_state.orientation[3] = _lowState.xingtian_state.orientation[3];
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];
    std_msgs::Float64MultiArray msg;
    for(int i(0); i < 12; ++i)
    {
        msg.data.push_back(_lowState.xingtian_state.wheel_force[i]);
    }
    _servo_pub[12].publish(msg);

}
template <typename T>
void IOROS<T>::initSend(){
    _servo_pub[0] = _nm.advertise<std_msgs::Float64>("/" + _robot_name + "/LF_hip_controller/command", 1);
    _servo_pub[1] = _nm.advertise<std_msgs::Float64>("/" + _robot_name + "/LF_knee_controller/command", 1);
    _servo_pub[2] = _nm.advertise<std_msgs::Float64>("/" + _robot_name + "/LF_wheel_controller/command", 1);
    _servo_pub[3] = _nm.advertise<std_msgs::Float64>("/" + _robot_name + "/LR_hip_controller/command", 1);
    _servo_pub[4] = _nm.advertise<std_msgs::Float64>("/" + _robot_name + "/LR_knee_controller/command", 1);
    _servo_pub[5] = _nm.advertise<std_msgs::Float64>("/" + _robot_name + "/LR_wheel_controller/command", 1);
    _servo_pub[6] = _nm.advertise<std_msgs::Float64>("/" + _robot_name + "/RR_hip_controller/command", 1);
    _servo_pub[7] = _nm.advertise<std_msgs::Float64>("/" + _robot_name + "/RR_knee_controller/command", 1);
    _servo_pub[8] = _nm.advertise<std_msgs::Float64>("/" + _robot_name + "/RR_wheel_controller/command", 1);
    _servo_pub[9] = _nm.advertise<std_msgs::Float64>("/" + _robot_name + "/RF_hip_controller/command", 1);
    _servo_pub[10] = _nm.advertise<std_msgs::Float64>("/" + _robot_name + "/RF_knee_controller/command", 1);
    _servo_pub[11] = _nm.advertise<std_msgs::Float64>("/" + _robot_name + "/RF_wheel_controller/command", 1);
    _servo_pub[12] = _nm.advertise<std_msgs::Float64MultiArray>("/" + _robot_name + "/visual/wheel_force", 1);
}
template <typename T>
void IOROS<T>::initRecv(){
    _imu_sub = _nm.subscribe("/imu", 1, &IOROS::imuCallback, this);
    _joint_states = _nm.subscribe("/" + _robot_name + "/joint_states", 1, &IOROS::jointStateCallback, this);
    _servo_sub[0] = _nm.subscribe("/" + _robot_name + "/torque_zqt_wheel", 1, &IOROS::LFwheelCallback, this);
    _servo_sub[1] = _nm.subscribe("/" + _robot_name + "/torque_zqt_knee", 1, &IOROS::LFkneeCallback, this);
    _servo_sub[2] = _nm.subscribe("/" + _robot_name + "/torque_zqt_hip", 1, &IOROS::LFhipCallback, this);
    _servo_sub[3] = _nm.subscribe("/" + _robot_name + "/torque_zht_wheel", 1, &IOROS::LRwheelCallback, this);
    _servo_sub[4] = _nm.subscribe("/" + _robot_name + "/torque_zht_knee", 1, &IOROS::LRkneeCallback, this);
    _servo_sub[5] = _nm.subscribe("/" + _robot_name + "/torque_zht_hip", 1, &IOROS::LRhipCallback, this);
    _servo_sub[6] = _nm.subscribe("/" + _robot_name + "/torque_yht_wheel", 1, &IOROS::RRwheelCallback, this);
    _servo_sub[7] = _nm.subscribe("/" + _robot_name + "/torque_yht_knee", 1, &IOROS::RRkneeCallback, this);
    _servo_sub[8] = _nm.subscribe("/" + _robot_name + "/torque_yht_hip", 1, &IOROS::RRhipCallback, this);
    _servo_sub[9] = _nm.subscribe("/" + _robot_name + "/torque_yqt_wheel", 1, &IOROS::RFwheelCallback, this);
    _servo_sub[10] = _nm.subscribe("/" + _robot_name + "/torque_yqt_knee", 1, &IOROS::RFkneeCallback, this);
    _servo_sub[11] = _nm.subscribe("/" + _robot_name + "/torque_yqt_hip", 1, &IOROS::RFhipCallback, this);
    // //model_stata
    // std::cout << "Model State Callback2" << std::endl;
    _servo_sub[12] = _nm.subscribe("/gazebo/model_states", 1, &IOROS::Model_stateCallback,this);  
    // std::cout << "Model State Callback3" << std::endl;
    // // laser_callback
    // _servo_sub[13] = _nm.subscribe("/xingtian/laser/scan",1,&IOROS::Laser_callback,this);
    // 相机订阅话题，留接口
    // _image_sub = 
}   
template <typename T>
void IOROS<T>::Model_stateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
   
     // 1. 检查模型是否存在
     if (msg->name.empty()) {
        ROS_WARN("Received empty model states!");
        return;
    }
    // 2. 查找特定模型的索引（例如 "xingtian"）
    int xingtian_index = -1;
    int ground_plane_index = -1;
    
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (std::string(msg->name[i]) == "xingtian") {
            xingtian_index = i;
        } else if (std::string(msg->name[i]) == "ground_plane") {
            ground_plane_index = i;
        }
    }
    // 3. 提取位姿和速度信息
    if (xingtian_index != -1) {
        // xingtian 的位姿
        geometry_msgs::Pose pose = msg->pose[xingtian_index];
        geometry_msgs::Twist twist = msg->twist[xingtian_index];
        _lowState.xingtian_state.position[0] = pose.position.x;
        _lowState.xingtian_state.position[1] = pose.position.y;
        _lowState.xingtian_state.position[2] = pose.position.z;

        _lowState.xingtian_state.vBody[0] = twist.linear.x;                 //世界坐标系下的速度
        _lowState.xingtian_state.vBody[1] = twist.linear.y;
        _lowState.xingtian_state.vBody[2] = twist.linear.z;

        _lowState.xingtian_state.omegaBody[0] = twist.angular.x;
        _lowState.xingtian_state.omegaBody[1] = twist.angular.y;
        _lowState.xingtian_state.omegaBody[2] = twist.angular.z;

        _lowState.xingtian_state.orientation[0] = pose.orientation.w;
        _lowState.xingtian_state.orientation[1] = pose.orientation.x;
        _lowState.xingtian_state.orientation[2] = pose.orientation.y;
        _lowState.xingtian_state.orientation[3] = pose.orientation.z;

        _lowState.xingtian_state.acceleration = _lowState.imu.accelerometer;
        // xingtian 的速度（线速度和角速度）
        // ROS_INFO("xingtian linear velocity: x=%.3f, y=%.3f, z=%.3f",
        //     pose.position.x, pose.position.y, pose.position.z);
        // ROS_INFO("xingtian linear velocity: x=%.3f, y=%.3f, z=%.3f",
        //          twist.linear.x, twist.linear.y, twist.linear.z);
        // ROS_INFO("xingtian angular velocity: x=%.3f, y=%.3f, z=%.3f",
        //          twist.angular.x, twist.angular.y, twist.angular.z);
    } else {
        ROS_WARN("Model 'xingtian' not found in model states!");
    }
    // // 可选：处理 ground_plane 的数据
    // if (ground_plane_index != -1) {
    //     // ground_plane 的位姿（通常为固定值）
    //     geometry_msgs::Pose pose = msg->pose[ground_plane_index];
    //     ROS_INFO("ground_plane position: z=%.3f", pose.position.z);
    // }
}




template <typename T>
void IOROS<T>::imuCallback(const sensor_msgs::Imu & msg)
{ 
    _lowState.imu.quaternion[0] = msg.orientation.w;
    _lowState.imu.quaternion[1] = msg.orientation.x;
    _lowState.imu.quaternion[2] = msg.orientation.y;
    _lowState.imu.quaternion[3] = msg.orientation.z;

    _lowState.imu.gyroscope[0] = msg.angular_velocity.x;
    _lowState.imu.gyroscope[1] = msg.angular_velocity.y;
    _lowState.imu.gyroscope[2] = msg.angular_velocity.z;
    
    _lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
    _lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
    _lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
    // std::cout << "IMU State Callback1" << std::endl;
    // _lowState.imu.linearvelocity[0] = msg.linear_acceleration.x;
    // _lowState.imu.linearvelocity[1] = msg.linear_acceleration.y;
    // _lowState.imu.linearvelocity[2] = msg.linear_acceleration.z;
}
template <typename T>
void IOROS<T>::jointStateCallback(const sensor_msgs::JointState & msg)
{
    //注意顺序！！！
    // 水平向前为0度，所有关节都是水平向前为0.逆时针为正。
    //顺时针为负，逆时针为正
    // 原始数据是从前往后依次为yht yqt zht zqt  wheel-knee-hip
    // 赋值之后是 LF LR RR RF HIP KNEE WHEEL
    for(int i(0); i < 12; ++i)
    {
        _lowState.motorState[i].mode = 1;
    }

    _lowState.motorState[0].q = msg.position[11];  
    _lowState.motorState[0].dq = msg.velocity[11];

    _lowState.motorState[1].q = msg.position[10];  
    _lowState.motorState[1].dq = msg.velocity[10];

    _lowState.motorState[2].q = msg.position[9];   
    _lowState.motorState[2].dq = msg.velocity[9];  //车轮    LF

    _lowState.motorState[3].q = msg.position[8];  
    _lowState.motorState[3].dq = msg.velocity[8];

    _lowState.motorState[4].q = msg.position[7];  
    _lowState.motorState[4].dq = msg.velocity[7];

    _lowState.motorState[5].q = msg.position[6];     //车轮   LR
    _lowState.motorState[5].dq = msg.velocity[6];

    _lowState.motorState[6].q = msg.position[2];  
    _lowState.motorState[6].dq = msg.velocity[2];

    _lowState.motorState[7].q = msg.position[1];  
    _lowState.motorState[7].dq = msg.velocity[1];

    _lowState.motorState[8].q = msg.position[0];      //车轮   RR
    _lowState.motorState[8].dq = msg.velocity[0];

    _lowState.motorState[9].q = msg.position[5];  
    _lowState.motorState[9].dq = msg.velocity[5];

    _lowState.motorState[10].q = msg.position[4];  
    _lowState.motorState[10].dq = msg.velocity[4];

    _lowState.motorState[11].q = msg.position[3];    //车轮   RF
    _lowState.motorState[11].dq = msg.velocity[3];


        // _lowState.motorState[i].ddq = msg.effort[i];  //这个扭矩读出来的是发送过去的扭矩数值

    
}
template <typename T>
void IOROS<T>::LFwheelCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    // 检查消息是否为空
    if (msg == nullptr) {
        ROS_ERROR("Received an empty WrenchStamped message.");
        return;
    }
    
    // 输出轮子的扭矩信息
    // ROS_INFO_STREAM("Left Front Wheel Torque:");



    // ROS_INFO_STREAM("Angular x: " << msg->wrench.torque.x << ", y: " << msg->wrench.torque.y << ", z: " << msg->wrench.torque.z);
    _lowState.motorState[2].tauEst = msg->wrench.torque.y + (msg->wrench.torque.x)*0.05 +(msg->wrench.torque.z)*0.05;
}
template <typename T>
void IOROS<T>::LFkneeCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    // 检查消息是否为空
    if (msg == nullptr) {
        ROS_ERROR("Received an empty WrenchStamped message.");
        return;
    } 
    _lowState.motorState[1].tauEst = msg->wrench.torque.y + (msg->wrench.torque.x)*0.05 +(msg->wrench.torque.z)*0.05;
} 
template <typename T>
void IOROS<T>::LFhipCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    // 检查消息是否为空
    if (msg == nullptr) {
        ROS_ERROR("Received an empty WrenchStamped message.");
        return;
    } 
    double q = _lowState.motorState[0].q; //关节绕y轴转动的角度
    // 输出轮子的力信息
    Mat3<T> R_y;
    R_y << cos(q), 0, sin(q),
        0,      1,      0,
        -sin(q), 0, cos(q);
    // ROS_INFO_STREAM("Left Front Wheel Force:");
    
    
    Vec3<T> force_in_joint_frame(msg->wrench.force.x, msg->wrench.force.y,msg->wrench.force.z);
    Vec3<T> force_in_fixed_frame = R_y * force_in_joint_frame;
    // ROS_INFO_STREAM("Linear x: " << msg->wrench.force.x << ", y: " << msg->wrench.force.y << ", z: " << msg->wrench.force.z);
    // ROS_INFO_STREAM("Fixed frame force - x: " << force_in_fixed_frame.x() 
    // << ", y: " << force_in_fixed_frame.y() 
    // << ", z: " << force_in_fixed_frame.z());
    _lowState.xingtian_state.wheel_force[0] = force_in_fixed_frame.x();
    _lowState.xingtian_state.wheel_force[1] = force_in_fixed_frame.y();
    _lowState.xingtian_state.wheel_force[2] = force_in_fixed_frame.z();
    _lowState.motorState[0].tauEst = msg->wrench.torque.y + (msg->wrench.torque.x)*0.05 +(msg->wrench.torque.z)*0.05;
} 
template <typename T>
void IOROS<T>::LRwheelCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    // 检查消息是否为空
    if (msg == nullptr) {
        ROS_ERROR("Received an empty WrenchStamped message.");
        return;
    }
    
    // ROS_INFO_STREAM("Angular x: " << msg->wrench.torque.x << ", y: " << msg->wrench.torque.y << ", z: " << msg->wrench.torque.z);
    _lowState.motorState[5].tauEst = msg->wrench.torque.y + (msg->wrench.torque.x)*0.05 +(msg->wrench.torque.z)*0.05;
}
template <typename T>
void IOROS<T>::LRkneeCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    // 检查消息是否为空
    if (msg == nullptr) {
        ROS_ERROR("Received an empty WrenchStamped message.");
        return;
    } 
    _lowState.motorState[4].tauEst = msg->wrench.torque.y + (msg->wrench.torque.x)*0.05 +(msg->wrench.torque.z)*0.05;
} 
template <typename T>
void IOROS<T>::LRhipCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    // 检查消息是否为空
    if (msg == nullptr) {
        ROS_ERROR("Received an empty WrenchStamped message.");
        return;
    } 
    double q = _lowState.motorState[3].q; //关节绕y轴转动的角度
    // 输出轮子的力信息
    Mat3<T> R_y;
    R_y << cos(q), 0, sin(q),
        0,      1,      0,
        -sin(q), 0, cos(q);
    // ROS_INFO_STREAM("Left Front Wheel Force:");
    
    
    Vec3<T> force_in_joint_frame(msg->wrench.force.x, msg->wrench.force.y,msg->wrench.force.z);
    Vec3<T> force_in_fixed_frame = R_y * force_in_joint_frame;
    // ROS_INFO_STREAM("Linear x: " << msg->wrench.force.x << ", y: " << msg->wrench.force.y << ", z: " << msg->wrench.force.z);
    // ROS_INFO_STREAM("Fixed frame force - x: " << force_in_fixed_frame.x() 
    // << ", y: " << force_in_fixed_frame.y() 
    // << ", z: " << force_in_fixed_frame.z());
    _lowState.xingtian_state.wheel_force[3] = force_in_fixed_frame.x();
    _lowState.xingtian_state.wheel_force[4] = force_in_fixed_frame.y();
    _lowState.xingtian_state.wheel_force[5] = force_in_fixed_frame.z();
    _lowState.motorState[3].tauEst = msg->wrench.torque.y + (msg->wrench.torque.x)*0.05 +(msg->wrench.torque.z)*0.05;
} 
template <typename T>
void IOROS<T>::RFwheelCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    // 检查消息是否为空
    if (msg == nullptr) {
        ROS_ERROR("Received an empty WrenchStamped message.");
        return;
    }
    
    // ROS_INFO_STREAM("Angular x: " << msg->wrench.torque.x << ", y: " << msg->wrench.torque.y << ", z: " << msg->wrench.torque.z);
    _lowState.motorState[11].tauEst = -(msg->wrench.torque.y*0.9 + (msg->wrench.torque.x)*0.05 +(msg->wrench.torque.z)*0.05);
}
template <typename T>
void IOROS<T>::RFkneeCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    // 检查消息是否为空
    if (msg == nullptr) {
        ROS_ERROR("Received an empty WrenchStamped message.");
        return;
    } 
    _lowState.motorState[10].tauEst = -(msg->wrench.torque.y*0.9 + (msg->wrench.torque.x)*0.05 +(msg->wrench.torque.z)*0.05);
} 
template <typename T>
void IOROS<T>::RFhipCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    // 检查消息是否为空
    if (msg == nullptr) {
        ROS_ERROR("Received an empty WrenchStamped message.");
        return;
    } 
    double q = _lowState.motorState[9].q; //关节绕y轴转动的角度
    // 输出轮子的力信息
    Mat3<T> R_y;
    R_y << cos(q), 0, sin(q),
        0,      1,      0,
        -sin(q), 0, cos(q);
    // ROS_INFO_STREAM("Left Front Wheel Force:");
    
    
    Vec3<T> force_in_joint_frame(msg->wrench.force.x, msg->wrench.force.y,msg->wrench.force.z);
    Vec3<T> force_in_fixed_frame = R_y * force_in_joint_frame;
    // ROS_INFO_STREAM("Linear x: " << msg->wrench.force.x << ", y: " << msg->wrench.force.y << ", z: " << msg->wrench.force.z);
    // ROS_INFO_STREAM("Fixed frame force - x: " << force_in_fixed_frame.x() 
    // << ", y: " << force_in_fixed_frame.y() 
    // << ", z: " << force_in_fixed_frame.z());
    _lowState.xingtian_state.wheel_force[9] = force_in_fixed_frame.x();
    _lowState.xingtian_state.wheel_force[10] = force_in_fixed_frame.y();
    _lowState.xingtian_state.wheel_force[11] = force_in_fixed_frame.z();
    _lowState.motorState[9].tauEst = -(msg->wrench.torque.y*0.9 + (msg->wrench.torque.x)*0.05 +(msg->wrench.torque.z)*0.05);
} 
template <typename T>
void IOROS<T>::RRwheelCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    // 检查消息是否为空
    if (msg == nullptr) {
        ROS_ERROR("Received an empty WrenchStamped message.");
        return;
    }
    
    // ROS_INFO_STREAM("Angular x: " << msg->wrench.torque.x << ", y: " << msg->wrench.torque.y << ", z: " << msg->wrench.torque.z);
    _lowState.motorState[8].tauEst = -(msg->wrench.torque.y*0.9 + (msg->wrench.torque.x)*0.05 +(msg->wrench.torque.z)*0.05);
}
template <typename T>
void IOROS<T>::RRkneeCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    // 检查消息是否为空
    if (msg == nullptr) {
        ROS_ERROR("Received an empty WrenchStamped message.");
        return;
    } 
    _lowState.motorState[7].tauEst = -(msg->wrench.torque.y*0.9 + (msg->wrench.torque.x)*0.05 +(msg->wrench.torque.z)*0.05);
} 
template <typename T>
void IOROS<T>::RRhipCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    // 检查消息是否为空
    if (msg == nullptr) {
        ROS_ERROR("Received an empty WrenchStamped message.");
        return;
    } 
    double q = _lowState.motorState[6].q; //关节绕y轴转动的角度
    // 输出轮子的力信息
    Mat3<T> R_y;
    R_y << cos(q), 0, sin(q),
        0,      1,      0,
        -sin(q), 0, cos(q);
    // ROS_INFO_STREAM("Left Front Wheel Force:");
    
    
    Vec3<T> force_in_joint_frame(msg->wrench.force.x, msg->wrench.force.y,msg->wrench.force.z);
    Vec3<T> force_in_fixed_frame = R_y * force_in_joint_frame;
    // ROS_INFO_STREAM("Linear x: " << msg->wrench.force.x << ", y: " << msg->wrench.force.y << ", z: " << msg->wrench.force.z);
    // ROS_INFO_STREAM("Fixed frame force - x: " << force_in_fixed_frame.x() 
    // << ", y: " << force_in_fixed_frame.y() 
    // << ", z: " << force_in_fixed_frame.z());
    _lowState.xingtian_state.wheel_force[6] = force_in_fixed_frame.x();
    _lowState.xingtian_state.wheel_force[7] = force_in_fixed_frame.y();
    _lowState.xingtian_state.wheel_force[8] = force_in_fixed_frame.z();
    _lowState.motorState[6].tauEst = -(msg->wrench.torque.y*0.9 + (msg->wrench.torque.x)*0.05 +(msg->wrench.torque.z)*0.05);
} 

template class IOROS<double>;
// template class IOROS<float>;
// int main(int argc, char **argv) 
// {

//     return 0;
// }
