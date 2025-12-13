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
#define MOTOR1_IN_A 25
#define MOTOR1_IN_B 26
// Правый
#define MOTOR2_IN_A 27
#define MOTOR2_IN_B 14

// ЭНКОДЕРЫ
// Левый
#define MOTOR1_ENC_A 18
#define MOTOR1_ENC_B 19
// Правый
#define MOTOR2_ENC_A 4  
#define MOTOR2_ENC_B 13

// ПИНЫ ДЛЯ СВЕТА
#define LED_PIN 2
#define LIGHT_PIN 15


// КАЛИБРОВКА
#define COUNTS_PER_REV 72150
#define WHEEL_DIAMETER 0.115   
#define LR_WHEELS_DISTANCE 0.54 
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