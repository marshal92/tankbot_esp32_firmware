#include "imu.h"

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
    
    return accel;
}

geometry_msgs__msg__Vector3 IMU::readGyroscope() {
    // ВАЖНО: Инициализируем нулями!
    geometry_msgs__msg__Vector3 gyro = {0, 0, 0};

    // Читаем и конвертируем deg/s -> rad/s
    gyro.x = myIMU.readFloatGyroX() * 0.0174533;
    gyro.y = myIMU.readFloatGyroY() * 0.0174533;
    gyro.z = myIMU.readFloatGyroZ() * 0.0174533;
    
    return gyro;
}