#ifndef AEE1A83F_66F2_45E5_AA56_CAC7F33C5FBF
#define AEE1A83F_66F2_45E5_AA56_CAC7F33C5FBF

#include "control/CtrlComponents.h"
template <typename T>
class XingtianParameters {
public:

    T gait_type;
    T gait_period_time;
    T gait_switching_phase;
    T gait_override;
    T gait_max_leg_angle;
    T gait_min_stance_time;
    T gait_max_stance_time;


};
// template class XingtianParameters<double>;
template class XingtianParameters<float>;



#endif /* AEE1A83F_66F2_45E5_AA56_CAC7F33C5FBF */
