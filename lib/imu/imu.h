#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include "SparkFunLSM6DS3.h" // Подключаем SparkFun
#include "Wire.h"
#include <geometry_msgs/msg/vector3.h>

class IMU {
    public:
        IMU();
        bool init(); 
        
        geometry_msgs__msg__Vector3 readAccelerometer();
        geometry_msgs__msg__Vector3 readGyroscope();

    private:
        LSM6DS3 myIMU; // Объект библиотеки /*  */ SparkFun
};

#endif