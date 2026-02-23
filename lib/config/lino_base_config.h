#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

// 1. ОПРЕДЕЛЯЕМ ТИПЫ РОБОТОВ
#define DIFFERENTIAL_DRIVE 0
#define SKID_STEER 1
#define MECANUM 2

// 2. ВЫБИРАЕМ ТИП РОБОТА
#define LINO_BASE DIFFERENTIAL_DRIVE 

// ПИНЫ МОТОРОВ
// Левый
#define MOTOR1_IN_A 4
#define MOTOR1_IN_B 5
// Правый
#define MOTOR2_IN_A 6
#define MOTOR2_IN_B 7

// ЭНКОДЕРЫ
// Левый
#define MOTOR1_ENC_A 8
#define MOTOR1_ENC_B 9
// Правый
#define MOTOR2_ENC_A 10
#define MOTOR2_ENC_B 11

// --- I2C (IMU) ---
// Выбрали GPIO 1 и 2 (они свободны и безопасны)
#define I2C_SDA 1
#define I2C_SCL 2

// --- СВЕТ ---
#define LED_PIN 48    // Встроенный RGB светодиод (или просто LED)
#define LIGHT_PIN 38  // Внешний свет

// КАЛИБРОВКА
#define COUNTS_PER_REV 71700
#define WHEEL_DIAMETER 0.115   
#define LR_WHEELS_DISTANCE 0.52 
#define MAX_RPM 160 


// PID НАСТРОЙКИ
#define K_P 0.4   
#define K_I 0.8
#define K_D 0.3 

// Ограничения выхода PID
#define PWM_MAX 255
#define PWM_MIN -255

#define PWM_START_MIN 50

// НАСТРОЙКИ МИНИМАЛЬНЫХ СКОРОСТЕЙ (HARD LIMITS)
#define MIN_ABS_LIN 0.2  
#define MIN_ABS_ANG 0.6

#endif