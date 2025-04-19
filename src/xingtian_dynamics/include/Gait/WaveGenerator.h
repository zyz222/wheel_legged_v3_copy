/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#ifndef A3BA11EE_3ED1_4F42_A79F_56B059C723C5
#define A3BA11EE_3ED1_4F42_A79F_56B059C723C5
#ifndef WAVEGENERATOR_H
#define WAVEGENERATOR_H

#include "common/cppTypes.h"
#include "common/timeMarker.h"
#include "common/enumClass.h"
#include <unistd.h>

#ifdef COMPILE_DEBUG
#include "common/PyPlot.h"
#endif  // COMPILE_DEBUG

/*generate linear wave, [0, 1]*/
template<typename T>
class WaveGenerator{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    WaveGenerator(T period, T stancePhaseRatio, Vec4<T> bias);
    ~WaveGenerator();
    void calcContactPhase(Vec4<T> &phaseResult, VecInt4 &contactResult, WaveStatus status);
    T getTstance();
    T getTswing();
    T getT();
private:
    void calcWave(Vec4<T> &phase, VecInt4 &contact, WaveStatus status);

    T _period;
    T _stRatio;
    Vec4<T> _bias;

    Vec4<T> _normalT;                   // [0, 1)
    Vec4<T> _phase, _phasePast;
    VecInt4 _contact, _contactPast;
    VecInt4 _switchStatus;          // 1: switching, 0: do not switch
    WaveStatus _statusPast;

    T _passT;                   // unit: second
    long long _startT;    // unit: us


};

#endif  // WAVEGENERATOR_H


#endif /* A3BA11EE_3ED1_4F42_A79F_56B059C723C5 */
