#include "pid.h"

PID::PID(float min_val, float max_val, float kp, float ki, float kd) 
    : min_val_(min_val), max_val_(max_val), kp_(kp), ki_(ki), kd_(kd) {
    integral_ = 0;
    prev_error_ = 0;
}

double PID::compute(float setpoint, float measured_value) {
    double error = setpoint - measured_value;

    // Пропорциональная часть
    double p_out = kp_ * error;

    // Интегральная часть
    integral_ += error;
    double i_out = ki_ * integral_;

    // Производная часть
    double derivative = error - prev_error_;
    double d_out = kd_ * derivative;

    prev_error_ = error;

    double output = p_out + i_out + d_out;

    // Ограничение выхода (Anti-windup для интеграла можно добавить позже, пока просто clamp)
    if (output > max_val_) output = max_val_;
    if (output < min_val_) output = min_val_;

    return output;
}

void PID::reset() {
    integral_ = 0;
    prev_error_ = 0;
}

void PID::updateConstants(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}