//  CIEM-ZYZ -2024
#include <ros/ros.h>
#include "interface/sim_gazebo_ros.h" // 包含 IOROS 类的头文件
#include "control/CtrlComponents.h"
#include "control/ControlFrame.h"

#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>

bool running = true;

// over watch the ctrl+c command
void ShutDown(int sig)
{
    std::cout << "stop the controller" << std::endl;
    running = false;
}
void setProcessScheduler() 
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}
int main(int argc, char **argv) {
    // 初始化 ROS
    ros::init(argc, argv, "xingtian_ros_node");

    // 创建 IOROS 实例
    IOROS ioros;

    // 创建 LowlevelCmd 和 LowlevelState 实例
    LowlevelCmd *cmd = new LowlevelCmd; 
    LowlevelState *state = new LowlevelState;

    // 这里需要根据您的实际需求来填充 cmd 和 state 的值
    // 示例初始化
    for (int i = 0; i < 12; ++i) {
        cmd->motorCmd[i].mode = 1; // 设置模式
        cmd->motorCmd[i].q = 0.0;   // 设置目标位置
        cmd->motorCmd[i].dq = 0.0;  // 设置目标速度
        cmd->motorCmd[i].tau = 10.5; // 设置目标扭矩
        cmd->motorCmd[i].Kd = 0.0;  // 设置微分增益
        cmd->motorCmd[i].Kp = 0.0;  // 设置比例增益
    }
// // RR   竖直方向为0，大腿、小腿向前摆动为负
//     cmd->motorCmd[1].q = -0;
//     cmd->motorCmd[2].q = -180 ;
// // RF   竖直方向为0，大腿、小腿向前摆动为负
//     cmd->motorCmd[4].q = -00;
//     cmd->motorCmd[5].q = -180 ;
// // LR   竖直方向为0，大腿、小腿向前摆动为负
//     cmd->motorCmd[7].q = -00;
//     cmd->motorCmd[8].q = -180 ;
// // LF   竖直方向为0，大腿、小腿向前摆动为负
//     cmd->motorCmd[10].q = -00;
//     cmd->motorCmd[11].q = -180 ;
    // 调用 sendRecv 函数
    while (ros::ok()) 
    {
        ioros.sendRecv(cmd,state);
        // 处理 state 中的数据
        // 例如，打印电机状态
        for (int i = 0; i < 12; ++i) {
            ROS_INFO("Motor %d: Position: %f, Velocity: %f, Estimated Torque: %f",
                     i, state->motorState[i].q, state->motorState[i].dq, state->motorState[i].tauEst);
        }
        // ros::spinOnce(); // 处理回调
        usleep(100000);  // 休眠100毫秒，控制循环频率
    }
    delete cmd;
    delete state;
    return 0;
}