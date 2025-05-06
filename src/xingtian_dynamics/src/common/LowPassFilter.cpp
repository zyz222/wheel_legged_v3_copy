/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "common/LowPassFilter.h"
#include <math.h>
//cutFrenquency: 截止频率
template <typename T>
LPFilter<T>::LPFilter(T samplePeriod, T cutFrequency){
    //weight 滤波权重，用于确定新输入值和历史值在当前输出中的占比
    _weight = 1.0 / ( 1.0 + 1.0/(2.0*M_PI * samplePeriod * cutFrequency) );
    _start  = false;
}
template <typename T>
void LPFilter<T>::addValue(T newValue){
    if(!_start){
        _start = true;
        _pastValue = newValue;
    }
    _pastValue = _weight*newValue + (1-_weight)*_pastValue;
}
template <typename T>
T LPFilter<T>::getValue(){
    return _pastValue;
}
template <typename T>
void LPFilter<T>::clear(){
    _start = false;
}
// template class LPFilter<double>;
template class LPFilter<float>;