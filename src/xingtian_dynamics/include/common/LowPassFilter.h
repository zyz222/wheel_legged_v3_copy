/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef LOWPASSFILTER
#define LOWPASSFILTER
template <typename T>
class LPFilter{
public:
    LPFilter(T samplePeriod, T cutFrequency);
    ~LPFilter();
    void addValue(T newValue);
    T getValue();
    void clear();
private:
    T _weight;
    T _pastValue;
    bool _start;
};

#endif  // LOWPASSFILTER