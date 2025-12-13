#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
    public:
        // Конструктор
        Motor(int pin_a, int pin_b);

        // Вращение
        void spin(int pwm);

    private:
        int pin_a_;
        int pin_b_;
        
        // В версии 2.x мы должны помнить номера каналов
        int channel_a_;
        int channel_b_;
        
        // Статическая переменная, чтобы моторы сами раздавали себе каналы (0, 1, 2...)
        static int channel_counter_;
};

#endif