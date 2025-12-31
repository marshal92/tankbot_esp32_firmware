#ifndef RAMP_H
#define RAMP_H

#include <Arduino.h>

class Ramp {
    float current_value;
    float max_acceleration; // Сколько добавляем за один цикл (шаг)

public:
    // step_size - это на сколько максимум может измениться скорость за один тик
    Ramp(float step_size) {
        current_value = 0.0;
        max_acceleration = step_size;
    }

    // Главная функция: получить сглаженное значение
    float update(float target) {
        float error = target - current_value;
        
        // Ограничиваем изменение скорости (магия плавности)
        float step = constrain(error, -max_acceleration, max_acceleration);
        
        current_value += step;
        return current_value;
    }

    // Мгновенный сброс (на случай экстренной остановки)
    void reset() {
        current_value = 0.0;
    }
    
    // Возможность поменять жесткость на лету
    void setStep(float step) {
        max_acceleration = step;
    }
};

#endif