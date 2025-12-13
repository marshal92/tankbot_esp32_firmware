#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <ESP32Encoder.h> // Библиотека madhephaestus

class Encoder {
public:
    Encoder(int pin_a, int pin_b, bool reverse);
    
    // Инициализация
    void init();
    
    // Получить текущее количество тиков (с момента старта)
    long read();
    
    // Сброс счетчика
    void write(long count);

private:
    int pin_a_;
    int pin_b_;
    bool reverse_;
    ESP32Encoder encoder_instance_;
};

#endif