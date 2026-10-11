//
// Created by lixin on 2026/10/11.
//

#ifndef RM5_PID_H
#define RM5_PID_H
class PID {
public:
    void init(float kp, float ki, float kd, float i_max,float out_max,float d_filter);
    void reset();
    float calc(float ref, float fdb);
private:
    //TODO
    float kp_, ki_, kd_;
    float i_max_ = 0.0f;//不太懂应该怎么给，先这样
    float out_max_ = 0.0f;//根据额定来的
    float d_filter_ = 1.0f;

    float err_;
    float integral_;
    float last_err_;
    float last_d_;

    float limit_out(float o);
    float limit_in(float i);
    float filter(float output);
};

#endif //RM5_PID_H
