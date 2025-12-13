#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>

// Структура для хранения оборотов моторов
struct rpm {
    float motor1;
    float motor2;
};

// Структура для хранения скоростей (для одометрии)
struct velocities {
    float linear_x;
    float linear_y;
    float angular_z;
};

class Kinematics {
    public:
        // Конструктор: принимает тип робота (из конфига), диаметр колес, ширину базы и макс. RPM
        Kinematics(int base, float wheel_diameter, float wheels_distance, int max_rpm);

        // Метод: Получить RPM для моторов из команды скорости (cmd_vel)
        rpm getRPM(float linear_x, float linear_y, float angular_z);

        // Метод: Получить реальную скорость робота из RPM (для одометрии - пригодится позже)
        velocities getVelocities(float rpm1, float rpm2);

    private:
        int base_;
        float wheel_diameter_;
        float wheels_distance_;
        int max_rpm_;
        
        // Вспомогательный метод расчета RPM
        float calculateRPM(float linear_x, float linear_y, float angular_z);
};

#endif