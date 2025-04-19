/*========================= Gamepad Control ==========================*/
/**
 * 将期望的指令转换为机器狗的状态指令
 */

#include "Controllers/DesiredStateCommand.h"
/*=========================== Gait Data ===============================*/
/**
 *
 */
template <typename T>
void DesiredStateData<T>::zero() {
  // Overall desired state
  if (reinterpret_cast<uintptr_t>(this) % 16 != 0) { // 检查对齐
    throw std::runtime_error("Misaligned this pointer!");
  }
  // std::cout<<"stateDes"<<  stateDes<<std::endl;
  stateDes = Vec12<T>::Zero();           //存储期望的线速度、角速度和位置和姿态
  stateTrajDes = Eigen::Matrix<T, 12, 10>::Zero();    //存储期望的状态轨迹！！
}
template struct DesiredStateData<double>;
template struct DesiredStateData<float>;

/**
 *
 */
template <typename T>
void DesiredStateCommand<T>::convertToStateCommands() {   //接收用户驱动指令，模式切换。通过操纵杆来驱动！！  主循环！

  data.zero();                 //存储期望状态和期望轨迹！！每次进来都会被清零重置！！！
  // stateDes = Vec12<T>::Zero();           //存储期望的线速度、角速度和位置和姿态
  // stateTrajDes = Eigen::Matrix<T, 12, 10>::Zero();
  Vec2<T>  joystickRight;     //左边操纵杆、右边操纵杆
  Vec2<T>  joystickLeft;
  //T height_cmd(0.3);
         //通过键盘来获取参数控制！！
  // joystickRight = Vec2<T>::Zero();
  joystickLeft =  (cmdPanel->getLeftStickAnalog().template cast<T>());       // 前进后退和转向
  joystickRight = (cmdPanel->getRightStickAnalog().template cast<T>());     //控制姿态roll-pitch
  height =  (cmdPanel->getHeight());    //获取机身高度指令

  joystickLeft[0] *= -1;    //方向都给它变反了？
  joystickRight[0] *= -1;

  leftAnalogStick = leftAnalogStick * (T(1) - filter) + joystickLeft * filter;    //T是转换类型的值    新输入的占0.1，当前值占0.9
  rightAnalogStick = rightAnalogStick * (T(1) - filter) + joystickRight * filter;
  //生成期望的状态
  // Desired states from the controller
  data.stateDes(0) += dt * data.stateDes(6);  // X position
  data.stateDes(1) += dt * data.stateDes(7);  // Y position
  data.stateDes(2) = height;  // Z position height
  
  data.stateDes(3) = saturation<T>((rightAnalogStick[0]),minRoll,maxRoll); // Roll
  data.stateDes(4) = saturation<T>((rightAnalogStick[1]), minPitch, maxPitch);  // Pitch
  // data.stateDes(5) = dt * data.stateDes(11);  // Yaw
  data.stateDes(5) = saturation<T>(atan2(data.stateDes(7),data.stateDes(6)), minTurnRate, maxTurnRate);  // Yaw turn rate

  data.stateDes(6) = saturation<T>((leftAnalogStick[0]), minVelX, maxVelX);  // forward linear velocity      //deadband 消除小范围内的噪声。
  data.stateDes(7) = saturation<T>((leftAnalogStick[1]), minVelY, maxVelY);  // lateral linear velocity  横向线速度
  // data.stateDes(7) = 0.0;                         //y方向速度
  data.stateDes(8) = 0.0;  // vertical linear velocity          
  
  data.stateDes(9) = 0.0;  // Roll rate
  data.stateDes(10) = 0.0;  // Pitch rate
  data.stateDes(11) = saturation<T>((leftAnalogStick[1])/(leftAnalogStick[0]), minTurnRate, maxTurnRate);  // Yaw turn rate
  // data.stateDes(11) = 0.0;
}

template <typename T>
void DesiredStateCommand<T>::setCommandLimits(T minVelX_in, T maxVelX_in,
    T minVelY_in, T maxVelY_in, T minTurnRate_in, T maxTurnRate_in) {
  minVelX = minVelX_in;
  maxVelX = maxVelX_in;
  minVelY = minVelY_in;
  maxVelY = maxVelY_in;
  minTurnRate = minTurnRate_in;
  maxTurnRate = maxTurnRate_in;
}

/**
 *
 */
template <typename T>   //这个函数好像没有调用？？？
void DesiredStateCommand<T>::desiredStateTrajectory(int N, Vec10<T> dtVec) {     //用于生成机器人未来的状态轨迹，基于当前的时间步长，预测未来N步的状态
  A = Mat12<T>::Zero();     //A是状态转移矩阵，用于预测未来N步的状态
  A(0, 0) = 1;
  A(1, 1) = 1;
  A(2, 2) = 1;
  A(3, 3) = 1;
  A(4, 4) = 1;
  A(5, 5) = 1;
  A(6, 6) = 1;
  A(7, 7) = 1;
  A(8, 8) = 1;
  A(9, 9) = 1;
  A(10, 10) = 1;
  A(11, 11) = 1;    
  data.stateTrajDes.col(0) = data.stateDes;   //状态轨迹的第一个状态就是当前状态

  for (int k = 1; k < N; k++) {     //预测N步状态      
    A(0, 6) = dtVec(k - 1);     //dtVec是时间步长向量，用于预测未来N步的状态
    A(1, 7) = dtVec(k - 1);
    A(2, 8) = dtVec(k - 1);
    A(3, 9) = dtVec(k - 1);
    A(4, 10) = dtVec(k - 1);
    A(5, 11) = dtVec(k - 1);
    data.stateTrajDes.col(k) = A * data.stateTrajDes.col(k - 1);   //输出每一步的步长。这个应该是没有用的！！！
    for (int i = 0; i < 12; i++) {
      // std::cout << data.stateTrajDes(i, k) << " ";
    }
    // std::cout << std::endl;
  }
  // std::cout << std::endl;
}

/**
 *
 */
template <typename T>
void DesiredStateCommand<T>::printStateCommandInfo() {
  // Increment printing iteration
  printIter++;

  // Print at requested frequency
  if (printIter == printNum) {
    std::cout << "[DESIRED STATE COMMAND] Printing State Command Info...\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << "Position X: " << data.stateDes(0)
              << " | Y: " << data.stateDes(1) << " | Z: " << data.stateDes(2)
              << "\n";
    std::cout << "Orientation Roll: " << data.stateDes(3)
              << " | Pitch: " << data.stateDes(4)
              << " | Yaw: " << data.stateDes(5) << "\n";
    std::cout << "Velocity X: " << data.stateDes(6)
              << " | Y: " << data.stateDes(7) << " | Z: " << data.stateDes(8)
              << "\n";
    std::cout << "Angular Velocity X: " << data.stateDes(9)
              << " | Y: " << data.stateDes(10) << " | Z: " << data.stateDes(11)
              << "\n";
    std::cout << std::endl;
    std::cout << std::endl;

    // Reset iteration counter
    printIter = 0;
  }
}
template class DesiredStateCommand<double>;
template class DesiredStateCommand<float>;
