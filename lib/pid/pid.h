#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float min_val, float max_val, float kp, float ki, float kd);
    
    // Расчет управляющего воздействия
    // setpoint - целевое значение (RPM)
    // measured_value - текущее значение (RPM)
    double compute(float setpoint, float measured_value);
    
    // Сброс интегральной ошибки (нужно при остановке)
    void reset();

    // Обновление коэффициентов "на лету" (если понадобится)
    void updateConstants(float kp, float ki, float kd);

private:
    float min_val_;
    float max_val_;
    float kp_;
    float ki_;
    float kd_;
    double integral_;
    double prev_error_;
};

#endif