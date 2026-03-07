#include "imu.h"
#include "lino_base_config.h"

#define REPORT_INTERVAL_US 5000 // 200 Hz

#ifndef IMU_RST_PIN
#define IMU_RST_PIN -1
#endif

IMU::IMU() : bno08x(IMU_RST_PIN) {
    accel_data.x = 0; accel_data.y = 0; accel_data.z = 0;
    gyro_data.x = 0; gyro_data.y = 0; gyro_data.z = 0;
    quat_data.x = 0; quat_data.y = 0; quat_data.z = 0; quat_data.w = 1.0;
}

bool IMU::init() {
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); // 400 kHz для быстрой шины

    // Пробуем адреса 0x4A и 0x4B
    if (!bno08x.begin_I2C(0x4A, &Wire, 0)) {
        if (!bno08x.begin_I2C(0x4B, &Wire, 0)) {
            return false;
        }
    }

    // Включаем репорты без магнитометра
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, REPORT_INTERVAL_US);
    bno08x.enableReport(SH2_ACCELEROMETER, REPORT_INTERVAL_US);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, REPORT_INTERVAL_US);

    return true;
}

bool IMU::update() {
    bool new_data = false;
    
    // Вычитываем все пришедшие пакеты
    while (bno08x.getSensorEvent(&sensorValue)) {
        new_data = true;
        switch (sensorValue.sensorId) {
            case SH2_GAME_ROTATION_VECTOR:
                quat_data.x = sensorValue.un.gameRotationVector.i;
                quat_data.y = sensorValue.un.gameRotationVector.j;
                quat_data.z = sensorValue.un.gameRotationVector.k;
                quat_data.w = sensorValue.un.gameRotationVector.real;
                break;
            case SH2_ACCELEROMETER:
                accel_data.x = sensorValue.un.accelerometer.x;
                accel_data.y = sensorValue.un.accelerometer.y;
                accel_data.z = sensorValue.un.accelerometer.z;
                break;
            case SH2_GYROSCOPE_CALIBRATED:
                gyro_data.x = sensorValue.un.gyroscope.x;
                gyro_data.y = sensorValue.un.gyroscope.y;
                gyro_data.z = sensorValue.un.gyroscope.z;
                break;
        }
    }
    return new_data;
}

geometry_msgs__msg__Vector3 IMU::readAccelerometer() { return accel_data; }
geometry_msgs__msg__Vector3 IMU::readGyroscope() { return gyro_data; }
geometry_msgs__msg__Quaternion IMU::readOrientation() { return quat_data; }