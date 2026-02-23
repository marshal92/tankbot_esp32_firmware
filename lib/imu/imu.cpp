#include "imu.h"
#include <math.h>
#include "lino_base_config.h"
// --- НАСТРОЙКИ СТАБИЛИЗАЦИИ ---
#define ACCEL_ALPHA 0.1   // Твой выбор (отклик быстрее)

// КАЛИБРОВКА 
#define GYRO_X_OFFSET  0.085
#define GYRO_Y_OFFSET -0.12 
#define GYRO_Z_OFFSET -0.074 

#define ACCEL_X_OFFSET -0.15
#define ACCEL_Y_OFFSET -0.23
#define ACCEL_Z_SCALE 0.98

// ЖЕСТКИЕ МЕРТВЫЕ ЗОНЫ (ЧИСТЫЙ НОЛЬ)
#define ACCEL_DEADZONE 0.08  // м/с^2
#define GYRO_DEADZONE  0.03  // рад/с

// Конструктор: убрали I2C_MODE, Adafruit инициализируется иначе
IMU::IMU() {
    filter_ax = 0; filter_ay = 0; filter_az = 0;
}

bool IMU::init() {
    // ВНИМАНИЕ: Укажи тут пины SDA и SCL, куда ты физически подключил датчик!
    // Для ESP32-S3 часто используют 8 и 9.
    Wire.begin(I2C_SDA, I2C_SCL);

    // Пытаемся найти MPU6050
    if (!myIMU.begin()) {
        return false;
    }

    // Настраиваем диапазоны для лучшей точности танка
    myIMU.setAccelerometerRange(MPU6050_RANGE_2_G);
    myIMU.setGyroRange(MPU6050_RANGE_250_DEG);
    myIMU.setFilterBandwidth(MPU6050_BAND_21_HZ); // Аппаратный фильтр от вибраций гусениц

    return true;
}

geometry_msgs__msg__Vector3 IMU::readAccelerometer() {
    geometry_msgs__msg__Vector3 accel;
    
    // Получаем новые данные от Adafruit
    sensors_event_t a, g, temp;
    myIMU.getEvent(&a, &g, &temp);

    // Данные УЖЕ в метрах на секунду в квадрате (м/с^2)! Не умножаем на 9.8!
    float raw_x = a.acceleration.x;
    float raw_y = a.acceleration.y;
    float raw_z = a.acceleration.z;

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
    if (fabs(filter_ax) < ACCEL_DEADZONE) filter_ax = 0.0;
    if (fabs(filter_ay) < ACCEL_DEADZONE) filter_ay = 0.0;
    
    accel.x = filter_ax;
    accel.y = filter_ay;
    accel.z = filter_az;
    
    return accel;
}

geometry_msgs__msg__Vector3 IMU::readGyroscope() {
    geometry_msgs__msg__Vector3 gyro = {0, 0, 0};

    // Получаем новые данные
    sensors_event_t a, g, temp;
    myIMU.getEvent(&a, &g, &temp);

    // Данные УЖЕ в радианах в секунду (рад/с)! Не умножаем на 0.017!
    float raw_x = g.gyro.x;
    float raw_y = g.gyro.y;
    float raw_z = g.gyro.z;

    // 1. Калибровка
    raw_x -= GYRO_X_OFFSET; 
    raw_y -= GYRO_Y_OFFSET;
    raw_z -= GYRO_Z_OFFSET; 

    // 2. МЕРТВАЯ ЗОНА
    if (fabs(raw_x) < GYRO_DEADZONE) raw_x = 0.0;
    if (fabs(raw_y) < GYRO_DEADZONE) raw_y = 0.0;
    if (fabs(raw_z) < 0.05)          raw_z = 0.0; 

    gyro.x = raw_x;
    gyro.y = raw_y;
    gyro.z = raw_z;
    
    return gyro;
}