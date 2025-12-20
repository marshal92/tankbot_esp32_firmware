#include "imu.h"
#include <math.h> // Нужно для функции fabs (модуль числа)

IMU::IMU() : myIMU(I2C_MODE, 0x6B) {
}

bool IMU::init() {
    Wire.begin(21, 22);
    if (myIMU.begin() != 0) {
        return false;
    }
    return true;
}

geometry_msgs__msg__Vector3 IMU::readAccelerometer() {
    // ВАЖНО: Инициализируем нулями!
    geometry_msgs__msg__Vector3 accel = {0, 0, 0};

    // Читаем и конвертируем g -> m/s^2
    accel.x = myIMU.readFloatAccelX() * 9.80665;
    accel.y = myIMU.readFloatAccelY() * 9.80665;
    accel.z = myIMU.readFloatAccelZ() * 9.80665;
    
    // Акселерометр пока оставляем "как есть", для 2D навигации он не критичен.
    // Если по Y будет сильный шум, можно добавить фильтр и сюда, но пока не надо.
    
    return accel;
}

geometry_msgs__msg__Vector3 IMU::readGyroscope() {
    // ВАЖНО: Инициализируем нулями!
    geometry_msgs__msg__Vector3 gyro = {0, 0, 0};

    // 1. Читаем сырые данные и переводим deg/s -> rad/s
    float raw_x = myIMU.readFloatGyroX() * 0.0174533;
    float raw_y = myIMU.readFloatGyroY() * 0.0174533;
    float raw_z = myIMU.readFloatGyroZ() * 0.0174533;

    // --- КОРРЕКЦИЯ ОСИ Z (Самое важное для карты!) ---
    
    // ШАГ А: Убираем постоянный дрейф (BIAS)
    // Твой датчик показывает примерно -0.03, когда стоит.
    // Мы прибавляем 0.03, чтобы "подтянуть" его к нулю.
    raw_z = raw_z + 0.03; 

    // ШАГ Б: Мертвая зона (DEADZONE)
    // Если скорость вращения меньше 0.05 рад/с (это около 3 градусов в секунду),
    // мы считаем, что робот стоит на месте. Это уберет дрожание карты.
    if (fabs(raw_z) < 0.05) {
        raw_z = 0.0;
    }

    // Записываем обработанные данные в сообщение
    gyro.x = raw_x;
    gyro.y = raw_y;
    gyro.z = raw_z;
    
    return gyro;
}