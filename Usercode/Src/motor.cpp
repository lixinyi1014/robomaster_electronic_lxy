//
// Created by lixin on 2026/10/11.
//

#include "motor.h"
#include "pid.h"

Motor::Motor(float sKp,float sKi,float sKd,
             float pKp,float pKi,float pKd,
             ControlMethod_e controlMethod) {
    spid_.init(sKp,sKi,sKd,0.0f);
    ppid_.init(pKp,pKi,pKd,0.0f);
    control_method_= controlMethod;
}

void Motor::handle() {
}

void Motor::setPosition(float target_position,float feedforward_speed = 0.0f,float feedforward_intensity = 0.0f) {
    target_speed_ = ppid_.calc(target_position,feedforward_intensity);
}
void setSpeed(float target_speed,float feedforward_intensity) {

}
void setIntensity(float intensity) {

}

