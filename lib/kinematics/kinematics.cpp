#include "kinematics.h"

Kinematics::Kinematics(int base, float wheel_diameter, float wheels_distance, int max_rpm) {
    base_ = base;
    wheel_diameter_ = wheel_diameter;
    wheels_distance_ = wheels_distance;
    max_rpm_ = max_rpm;
}

rpm Kinematics::getRPM(float linear_x, float linear_y, float angular_z) {
    float tangential_vel = angular_z * (wheels_distance_ / 2.0);

    // Линейная скорость переводится в м/с для каждого колеса
    // Для танка (Differential Drive):
    // Левое = V - (W * d / 2)
    // Правое = V + (W * d / 2)
    float linear_vel_x_mins = linear_x - tangential_vel;
    float linear_vel_x_plus = linear_x + tangential_vel;

    // Переводим м/с в RPM
    // RPM = (v * 60) / (pi * D)
    float m1_rpm = (linear_vel_x_mins * 60.0) / (PI * wheel_diameter_);
    float m2_rpm = (linear_vel_x_plus * 60.0) / (PI * wheel_diameter_);

    // Ограничиваем макс скоростью мотора
    m1_rpm = constrain(m1_rpm, -max_rpm_, max_rpm_);
    m2_rpm = constrain(m2_rpm, -max_rpm_, max_rpm_);

    return {m1_rpm, m2_rpm};
}

velocities Kinematics::getVelocities(float rpm1, float rpm2) {
    velocities vel;
    // Средняя скорость колес в м/с
    float average_rps_x = ((float)(rpm1 + rpm2) / 2.0) / 60.0;
    vel.linear_x = average_rps_x * wheel_diameter_ * PI;
    
    vel.linear_y = 0; // Танк боком не едет

    // Угловая скорость
    float average_rps_a = ((float)(rpm2 - rpm1) / 2.0) / 60.0;
    vel.angular_z = (average_rps_a * wheel_diameter_ * PI) / (wheels_distance_ / 2.0);

    return vel;
}