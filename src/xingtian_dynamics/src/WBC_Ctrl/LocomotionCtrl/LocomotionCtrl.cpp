

#include "WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp"
#include <WBC_Ctrl/ContactSet/SingleContact.hpp>
#include <WBC_Ctrl/TaskSet/BodyOriTask.hpp>
#include <WBC_Ctrl/TaskSet/BodyPosTask.hpp>
#include <WBC_Ctrl/TaskSet/LinkPosTask.hpp>

template<typename T>
LocomotionCtrl<T>::LocomotionCtrl(QuadrupedRobot<T> model):
  WBC_Ctrl<T>(model)
{
  _body_pos_task = new BodyPosTask<T>(&(WBCtrl::_model));              //这里传入了参数，更新了基类的数据
  _body_ori_task = new BodyOriTask<T>(&(WBCtrl::_model));               


  _foot_contact[0] = new SingleContact<T>(&(WBCtrl::_model), linkID::FL);   //足端接触点
  _foot_contact[1] = new SingleContact<T>(&(WBCtrl::_model), linkID::HL);    
  _foot_contact[2] = new SingleContact<T>(&(WBCtrl::_model), linkID::HR);
  _foot_contact[3] = new SingleContact<T>(&(WBCtrl::_model), linkID::FR);

  _foot_task[0] = new LinkPosTask<T>(&(WBCtrl::_model), linkID::FL);   //连杆的id,足端
  _foot_task[1] = new LinkPosTask<T>(&(WBCtrl::_model), linkID::HL);
  _foot_task[2] = new LinkPosTask<T>(&(WBCtrl::_model), linkID::HR);
  _foot_task[3] = new LinkPosTask<T>(&(WBCtrl::_model), linkID::FR);
}

template<typename T>
LocomotionCtrl<T>::~LocomotionCtrl(){
  delete _body_pos_task;
  delete _body_ori_task;

  for(size_t i (0); i<4; ++i){
    delete _foot_contact[i];
    delete _foot_task[i];
  }
}
//运动控制
template<typename T>     //data,机器人状态信息     void指针，需要转换数据类型
void LocomotionCtrl<T>::_ContactTaskUpdate(void* input, CtrlComponents<T> & data){             //平衡站立控制中调用这个。接触相任务的更新
  _input_data = static_cast<LocomotionCtrlData<T>* >(input);    //输入数据指针，包含了所有的机器人运动控制期望值和状态信息，还有期望的接触力！！

  _ParameterSetup(data.lowCmd);           //更新Kp,和Kd数值，任务优先级，控制增益等
  
  // Wash out the previous setup
  _CleanUp();                                       //清除掉所有的任务序列
  // 输入的期望姿态！！
  _quat_des = ori::rpyToQuat(_input_data->pBody_RPY_des);            //把输入的RPY转成四元数，作为期望的姿态

  Vec3<T> zero_vec3; zero_vec3.setZero();
  //有两个任务！   调用基类的成员函数
  _body_ori_task->UpdateTask(&_quat_des, _input_data->vBody_Ori_des, zero_vec3);         //更新姿态控制任务，设置期望的四元数姿态，姿态速度和加速度。
  _body_pos_task->UpdateTask(                                  //更新位置控制任务，设置期望的位置，速度和加速度。
      &(_input_data->pBody_des), 
      _input_data->vBody_des, 
      _input_data->aBody_des);
  //push_back，在容器末尾添加一个任务
  WBCtrl::_task_list.push_back(_body_ori_task);                //把任务添加到任务序列中
  WBCtrl::_task_list.push_back(_body_pos_task);               

  for(size_t leg(0); leg<4; ++leg){                                       //四个腿的！
    if(_input_data->contact_state[leg] > 0.){ // Contact                   //如果接触了，就设置接触力，更新接触任务，把接触任务添加到任务序列中
      _foot_contact[leg]->setRFDesired((DVec<T>)(_input_data->Fr_des[leg]));     //设置期望的接触力
      _foot_contact[leg]->UpdateContactSpec();                                   //更新接触规划，例如力的限制，摩擦系数等
      WBCtrl::_contact_list.push_back(_foot_contact[leg]);                          //把接触任务添加到接触序列中

    }else{ // No Contact (swing)
      _foot_task[leg]->UpdateTask(                                                  //更新足端位置控制任务，设置期望的位置，速度和加速度。
          &(_input_data->pFoot_des[leg]), 
          _input_data->vFoot_des[leg], 
          _input_data->aFoot_des[leg]);
          //zero_vec3);
      WBCtrl::_task_list.push_back(_foot_task[leg]);
    }
  }
}


template<typename T>
void LocomotionCtrl<T>::_ParameterSetup(const LowlevelCmd<T>* param){

  for(size_t i(0); i<3; ++i){                      //三个自由度的！
    ((BodyPosTask<T>*)_body_pos_task)->_Kp[i] = param->task_kpkd.Kp_body[i];
    ((BodyPosTask<T>*)_body_pos_task)->_Kd[i] = param->task_kpkd.Kd_body[i];

    ((BodyOriTask<T>*)_body_ori_task)->_Kp[i] = param->task_kpkd.Kp_ori[i];
    ((BodyOriTask<T>*)_body_ori_task)->_Kd[i] = param->task_kpkd.Kd_ori[i];

    for(size_t j(0); j<4; ++j){   
      ((LinkPosTask<T>*)_foot_task[j])->_Kp[i] = param->task_kpkd.Kp_foot[i];
      ((LinkPosTask<T>*)_foot_task[j])->_Kd[i] = param->task_kpkd.Kd_foot[i];
      //((LinkPosTask<T>*)_foot_task[j])->_Kp_kin[i] = 1.5;
    }

    WBCtrl::_Kp_joint[i] = param->task_kpkd.Kp_joint[i];
    WBCtrl::_Kd_joint[i] = param->task_kpkd.Kd_joint[i];

   }
}
template<typename T>
void LocomotionCtrl<T>::_CleanUp(){
  WBCtrl::_contact_list.clear();
  WBCtrl::_task_list.clear();
}
// template class LocomotionCtrl<double>;
template class LocomotionCtrl<float>;

