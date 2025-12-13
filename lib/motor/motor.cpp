#include "motor.h"
#include "lino_base_config.h"

// Инициализируем счетчик каналов с 0
int Motor::channel_counter_ = 0;

Motor::Motor(int pin_a, int pin_b) {
    pin_a_ = pin_a;
    pin_b_ = pin_b;

    // Назначаем каналы автоматически:
    // Первый мотор возьмет каналы 0 и 1
    // Второй мотор возьмет каналы 2 и 3
    channel_a_ = channel_counter_++;
    channel_b_ = channel_counter_++;

    pinMode(pin_a_, OUTPUT);
    pinMode(pin_b_, OUTPUT);

    // === ЛОГИКА ДЛЯ ЯДРА 2.x (Ваша текущая версия) ===
    
    // 1. Настраиваем каналы (Частота 5000 Гц, 8 бит)
    ledcSetup(channel_a_, 20000, 8);
    ledcSetup(channel_b_, 20000, 8);

    // 2. Привязываем каналы к пинам
    ledcAttachPin(pin_a_, channel_a_);
    ledcAttachPin(pin_b_, channel_b_);
}

void Motor::spin(int pwm) {
    
    // === ЛОГИКА МЕРТВОЙ ЗОНЫ (FEEDFORWARD) ===
    // Берем значение 80 (или сколько ты поставил) из конфига
    
    if (pwm > 0) 
    {
        // Едем ВПЕРЕД: добавляем "пинок" к тому, что просит PID
        pwm += PWM_START_MIN; 
        
        // Ограничиваем, чтобы не улететь за 255
        if (pwm > 255) pwm = 255;
    }
    else if (pwm < 0) 
    {
        // Едем НАЗАД: вычитаем "пинок"
        pwm -= PWM_START_MIN;
        
        // Ограничиваем
        if (pwm < -255) pwm = -255;
    }
    // Если pwm == 0, ничего не добавляем, стоим.

// === УПРАВЛЕНИЕ ДРАЙВЕРОМ (BTS7960) ===
    if (pwm > 0) {
        // ВПЕРЕД
        ledcWrite(channel_a_, abs(pwm));
        ledcWrite(channel_b_, 0);
    } else if (pwm < 0) {
        // НАЗАД
        ledcWrite(channel_a_, 0);
        ledcWrite(channel_b_, abs(pwm));
    } else {
        // СТОП (Тормоз)
        ledcWrite(channel_a_, 0);
        ledcWrite(channel_b_, 0);
    }
}