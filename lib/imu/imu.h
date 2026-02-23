#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <geometry_msgs/msg/vector3.h>

class IMU {
    public:
        IMU();
        bool init(); 
        
        geometry_msgs__msg__Vector3 readAccelerometer();
        geometry_msgs__msg__Vector3 readGyroscope();

    private:
        Adafruit_MPU6050 myIMU; // Об'єкт для MPU6050
        float filter_ax, filter_ay, filter_az;
};

#endif