#include "imu.h"
#include <math.h>

// --- НАСТРОЙКИ СТАБИЛИЗАЦИИ ---

#define ACCEL_ALPHA 0.1   // Твой выбор (отклик быстрее)

// КАЛИБРОВКА (Подправлена по последнему видео 1000008541.mp4)
#define GYRO_X_OFFSET  0.085
#define GYRO_Y_OFFSET -0.12 
#define GYRO_Z_OFFSET -0.074 

#define ACCEL_X_OFFSET -0.15
#define ACCEL_Y_OFFSET -0.23
#define ACCEL_Z_SCALE 0.98

// ЖЕСТКИЕ МЕРТВЫЕ ЗОНЫ (ЧИСТЫЙ НОЛЬ)
// Если значение меньше этого порога, оно становится 0.00
// Это уберет "дрожание" цифр в статике.
#define ACCEL_DEADZONE 0.08  // м/с^2 (Примерно 0.006 g)
#define GYRO_DEADZONE  0.03  // рад/с (Примерно 1 градус/сек)

IMU::IMU() : myIMU(I2C_MODE, 0x6B) {
    filter_ax = 0; filter_ay = 0; filter_az = 0;
}

bool IMU::init() {
    Wire.begin(21, 22);
    return (myIMU.begin() == 0);
}

geometry_msgs__msg__Vector3 IMU::readAccelerometer() {
    geometry_msgs__msg__Vector3 accel;

    float raw_x = myIMU.readFloatAccelX() * 9.80665;
    float raw_y = myIMU.readFloatAccelY() * 9.80665;
    float raw_z = myIMU.readFloatAccelZ() * 9.80665;

    // 1. Калибровка
    raw_x -= ACCEL_X_OFFSET; 
    raw_y -= ACCEL_Y_OFFSET; 
    raw_z *= ACCEL_Z_SCALE; 

    // 2. Фильтрация (LPF)
    if (filter_ax == 0) { 
        filter_ax = raw_x; filter_ay = raw_y; filter_az = raw_z; 
    } else {
        filter_ax = (ACCEL_ALPHA * raw_x) + ((1.0 - ACCEL_ALPHA) * filter_ax);
        filter_ay = (ACCEL_ALPHA * raw_y) + ((1.0 - ACCEL_ALPHA) * filter_ay);
        filter_az = (ACCEL_ALPHA * raw_z) + ((1.0 - ACCEL_ALPHA) * filter_az);
    }

    // 3. МЕРТВАЯ ЗОНА (Железный ноль)
    // Если мы стоим ровно (шум меньше 0.06), пишем 0.0
    if (fabs(filter_ax) < ACCEL_DEADZONE) filter_ax = 0.0;
    if (fabs(filter_ay) < ACCEL_DEADZONE) filter_ay = 0.0;
    
    // Z не трогаем deadzone-ом, там гравитация!

    accel.x = filter_ax;
    accel.y = filter_ay;
    accel.z = filter_az;
    
    return accel;
}

geometry_msgs__msg__Vector3 IMU::readGyroscope() {
    geometry_msgs__msg__Vector3 gyro = {0, 0, 0};

    float raw_x = myIMU.readFloatGyroX() * 0.0174533;
    float raw_y = myIMU.readFloatGyroY() * 0.0174533;
    float raw_z = myIMU.readFloatGyroZ() * 0.0174533;

    // 1. Калибровка
    raw_x -= GYRO_X_OFFSET; 
    raw_y -= GYRO_Y_OFFSET;
    raw_z -= GYRO_Z_OFFSET; 

    // 2. МЕРТВАЯ ЗОНА ДЛЯ ВСЕХ ОСЕЙ
    // Если вращение мизерное - считаем, что стоим намертво.
    if (fabs(raw_x) < GYRO_DEADZONE) raw_x = 0.0;
    if (fabs(raw_y) < GYRO_DEADZONE) raw_y = 0.0;
    if (fabs(raw_z) < 0.05)          raw_z = 0.0; // Для Z порог повыше (твое старое значение)

    gyro.x = raw_x;
    gyro.y = raw_y;
    gyro.z = raw_z;
    
    return gyro;
}