//
// Created by lixin on 2026/10/11.
//

#ifndef RM5_PID_H
#define RM5_PID_H
class PID {
public:
    PID(float kp, float ki, float kd, float i_max,
        float out_max, float d_filter);
    void reset();
    float calc(float ref, float fdb);
private:
    //TODO
    float kp_, ki_, kd_;
    float i_max_;
    float out_max_;
    float d_filter_;

    float err_;
    float last_err_;
    float err_sum_;;

    float limit_out(float o);
    float limit_in(float i);
};

#endif //RM5_PID_H
