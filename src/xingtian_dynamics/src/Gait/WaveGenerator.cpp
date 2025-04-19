/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
//波形发生器
// 后期在这里修改 触地检测的代码！！！！！
// 以及步态状态的修改

// 用于机器人运动控制中的步态生成模块，可能是四足机器人行走的核心部分。
// 它的功能是生成四条腿的相位（phase）和接触状态（contact），并根据步态的状态（WaveStatus）更新运动参数。

// 判断是否接触
#include "Gait/WaveGenerator.h"
#include <iostream>
#include <sys/time.h>
#include <math.h>
#include <cmath>

//period: 步态周期，stancePhaseRatio: stancePhase占步态周期的比例，   bias: 相位偏移
//好像没调用
template <typename T>
WaveGenerator<T>::WaveGenerator(T period, T stancePhaseRatio, Vec4<T> bias)
                :_period(period), _stRatio(stancePhaseRatio), _bias(bias)
{

    if ((_stRatio >= 1) || (_stRatio <= 0))
    {
        std::cout << "[ERROR] The stancePhaseRatio of WaveGenerator should between (0, 1)" << std::endl;
        exit(-1);
    }

    for (int i(0); i < bias.rows(); ++i)
    {
        if ((bias(i) > 1) || (bias(i) < 0))
        {
            std::cout << "[ERROR] The bias of WaveGenerator should between [0, 1]" << std::endl;
            exit(-1);
        }
    }

    _startT = getSystemTime();            //获取当前时间
    _contactPast.setZero();                 //上一时刻接触状态，触地
    _phasePast << 0.5, 0.5, 0.5, 0.5;   //上一时刻的相位状态
    _statusPast = WaveStatus::SWING_ALL;    //上一时刻的步态状态
}
template <typename T>
WaveGenerator<T>::~WaveGenerator()
{
}
//计算接触相位和接触状态
//status 步态状态 //这里还需要修改！！！！
// 目前是根据是否摆动和站立的情况来计算的摆动相
// 利用上一时刻的状态来判断是否需要平滑过渡，最终返回当前的状态！
template <typename T>
void WaveGenerator<T>::calcContactPhase(Vec4<T> &phaseResult, VecInt4 &contactResult, WaveStatus status)
{

    calcWave(_phase, _contact, status);   //相位、接触状态、步态状态，计算当前的！！！

    if (status != _statusPast)     //判断是否发生状态切换！！ 如果状态不一致了 全支撑到全摆动 切换到这里
    {
        if (_switchStatus.sum() == 0)   //判断所有腿都没有切换状态
        {
            _switchStatus.setOnes();     //所有腿都没有切换状态,标记状态需要更新
        }
        calcWave(_phasePast, _contactPast, _statusPast); //根据过去的相位和接触状态以及状态来计算腿部状态
        // 过去状态的计算是为了比较当前状态与过去状态的差异，从而决定哪些腿需要更新状态，哪些腿可以保持不变。
        // two special case
        if ((status == WaveStatus::STANCE_ALL) && (_statusPast == WaveStatus::SWING_ALL))
        {
            _contactPast.setOnes();    //全触地为1
        }
        else if ((status == WaveStatus::SWING_ALL) && (_statusPast == WaveStatus::STANCE_ALL))
        {
            _contactPast.setZero();   //摆动为0
        }
    }
    // 如果发生了切换，利用过去的状态进行处理
    if (_switchStatus.sum() != 0)        //有腿切换了状态！！
    {
        for (int i(0); i < 4; ++i)
        {
            if (_contact(i) == _contactPast(i))  //判断接触状态是否与过去相同
            {
                _switchStatus(i) = 0;            //0表示不切换
            }
            else                                 //状态没有切换，但是接触相变了
            {
                _contact(i) = _contactPast(i);
                _phase(i) = _phasePast(i);
            }
        }
        if (_switchStatus.sum() == 0)              //状态没有切换 0 表示不切换
        {
            _statusPast = status;
        }
    }
    phaseResult = _phase;
    contactResult = _contact;
}
//获取支撑相时间
template <typename T>
T WaveGenerator<T>::getTstance()
{
    return _period * _stRatio;
}
//获取摆动相时间
template <typename T>
T WaveGenerator<T>::getTswing()
{
    return _period * (1 - _stRatio);
}
//获取周期时间
template <typename T>
T WaveGenerator<T>::getT()
{
    return _period;
}
//根据当前时间和状态计算步态相位和接触状态，在这里更改了相位接触状态，后续是需要修改的！！！！
template <typename T>
void WaveGenerator<T>::calcWave(Vec4<T> &phase, VecInt4 &contact, WaveStatus status)
{
    if (status == WaveStatus::WAVE_ALL)   //波动
    {
        // 这个是按时间来计算是否接触和摆动
        // 后续在这里可以继续修改！！按照概率来计算！
        _passT = (T)(getSystemTime() - _startT) * 1e-6; //获取过去的时间
        for (int i(0); i < 4; ++i)
        {
            //bias是第i条腿的偏移时间 fmod是取余数
            //每条腿的时间先后顺序？
            // std::cout << "过去的时间" << _passT << std::endl;
            _normalT(i) = fmod(_passT + _period - _period * _bias(i), _period) / _period;
            if (_normalT(i) < _stRatio)   //）stRatio是stance phase的百分比
            {
                contact(i) = 1;
                // std::cout << "第i条腿触地" << i << std::endl;
                phase(i) = _normalT(i) / _stRatio;  //计算每条腿在当前周期中的相位，即当前腿在支撑相阶段的进展度
            }
            else
            {
                contact(i) = 0;
                phase(i) = (_normalT(i) - _stRatio) / (1 - _stRatio);
            }
        }
    }
    else if (status == WaveStatus::SWING_ALL)
    {
        contact.setZero();
        phase << 0.5, 0.5, 0.5, 0.5; //触地系数，占这条腿周期的比例
    }
    else if (status == WaveStatus::STANCE_ALL)
    {
        contact.setOnes();
        phase << 0.5, 0.5, 0.5, 0.5;
    }
}
// template class WaveGenerator<float>;
template class WaveGenerator<double>;