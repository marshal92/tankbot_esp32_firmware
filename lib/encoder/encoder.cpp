#include "encoder.h"

Encoder::Encoder(int pin_a, int pin_b, bool reverse) {
    pin_a_ = pin_a;
    pin_b_ = pin_b;
    reverse_ = reverse;
}

void Encoder::init() {
    // Включаем PCNT модуль для этого экземпляра
    // ESP32Encoder автоматически выбирает свободный юнит PCNT
    ESP32Encoder::useInternalWeakPullResistors = puType::up; // Включаем внутреннюю подтяжку
    encoder_instance_.attachHalfQuad(pin_a_, pin_b_); // Подключаем пины
    encoder_instance_.clearCount();
}

long Encoder::read() {
    long value = encoder_instance_.getCount();
    return reverse_ ? -value : value;
}

void Encoder::write(long count) {
    encoder_instance_.setCount(reverse_ ? -count : count);
}