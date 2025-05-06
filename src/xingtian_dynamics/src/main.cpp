//  CIEM-ZYZ -2024
#include <ros/ros.h>
#include "interface/sim_gazebo_ros.h" // 包含 IOROS 类的头文件

#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"
#include "Gait/WaveGenerator.h"
#include "control/BalanceCtrl.h"
#include "interface/KeyBoard.h"
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#define RESET   "\033[0m"   // 重置颜色
#define RED     "\033[31m"  // 红色
#define GREEN   "\033[32m"  // 绿色
#define YELLOW  "\033[33m"  // 黄色
#define BLUE    "\033[34m"  // 蓝色
#define MAGENTA "\033[35m"  // 品红
#define CYAN    "\033[36m"  // 青色
#define WHITE   "\033[37m"  // 白色
#define BOLD    "\033[1m"   // 加粗
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
    setProcessScheduler();
    /* set the print format */
    std::cout << std::fixed << std::setprecision(3);

    IOInterface<float> *ioInter;
    CtrlPlatform ctrlPlat;

    ioInter = new IOROS<float>();
    ctrlPlat = CtrlPlatform::GAZEBO;

    CtrlComponents<float> *ctrlComp = new CtrlComponents<float>(ioInter);  //控制部件
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 0.002; // run at 500hz
    ctrlComp->running = &running;
    
    ctrlComp->robotModel = new xingtianRobot<float>();

    ctrlComp->waveGen = new WaveGenerator<float>(0.45, 0.5, Vec4<float>(0.5, 0, 0.5, 0)); // Trot  RF LF RR LR
    // ctrlComp->waveGen = new WaveGenerator(0.45, 0.5, Vec4(0.5, 0, 0, 0)); //
    // ctrlComp->waveGen = new WaveGenerator(0.5, 0.5, Vec4(0, 0, 0, 0)); // Trot
    // ctrlComp->waveGen = new WaveGenerator(1.1, 0.75, Vec4(0, 0.25, 0.5, 0.75));  //Crawl, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.6, Vec4(0, 0.5, 0.5, 0));  //Walking Trot, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.35, Vec4(0, 0.5, 0.5, 0));  //Running Trot, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.7, Vec4(0, 0, 0, 0));  //Pronk, only for sim
    ctrlComp->geneObj();
 
    

    ControlFrame<float> ctrlFrame(ctrlComp);

    signal(SIGINT, ShutDown);

    while (running)
    {
        ctrlFrame.run();
    }

    delete ctrlComp;
 


#ifdef COMPILE_DEBUG_TEST
    // // 创建 IOROS 实例
    IOROS ioros;

    // // 创建 LowlevelCmd 和 LowlevelState 实例
    LowlevelCmd *cmd = new LowlevelCmd; 
    LowlevelState *state = new LowlevelState;

    // 这里需要根据您的实际需求来填充 cmd 和 state 的值
    // 示例初始化
    for (int i = 0; i < 12; ++i) {
        cmd->motorCmd[i].mode = 1; // 设置模式
        cmd->motorCmd[i].q = 0.0;   // 设置目标位置
        cmd->motorCmd[i].dq =0.0;  // 设置目标速度
        cmd->motorCmd[i].tau = -0.0; // 设置前馈扭矩  ，必须有，没有就会乱动
        cmd->motorCmd[i].Kd = 00.5;  // 设置微分增益
        cmd->motorCmd[i].Kp = 5000.0;  // 设置比例增益
    }
// RR   竖直方向为0，大腿、小腿向前摆动为负
    cmd->motorCmd[1].q = -0;
    cmd->motorCmd[2].q = -180 ;
// RF   竖直方向为0，大腿、小腿向前摆动为负
    cmd->motorCmd[4].q = -00;
    cmd->motorCmd[5].q = -180 ;
// LR   竖直方向为0，大腿、小腿向前摆动为负
    cmd->motorCmd[7].q = -00;
    cmd->motorCmd[8].q = -180 ;
// LF   竖直方向为0，大腿、小腿向前摆动为负
    cmd->motorCmd[10].q = -00;
    cmd->motorCmd[11].q = -180 ;


    // 调用 sendRecv 函数
    while (ros::ok()) 
    {
        ioros.sendRecv(cmd,state);
        // 处理 state 中的数据
        // RR RF LR LF 顺序不能错， wheel-knee-hip！！！！
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
#endif //COMPILE_DEBUG

    return 0;
}