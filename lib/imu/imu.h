#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/quaternion.h> 

class IMU {
    public:
        IMU();
        bool init(); 
        
        bool update(); 
        
        geometry_msgs__msg__Vector3 readAccelerometer();
        geometry_msgs__msg__Vector3 readGyroscope();
        geometry_msgs__msg__Quaternion readOrientation(); 

    private:
        Adafruit_BNO08x bno08x; 
        sh2_SensorValue_t sensorValue;

        geometry_msgs__msg__Vector3 accel_data;
        geometry_msgs__msg__Vector3 gyro_data;
        geometry_msgs__msg__Quaternion quat_data;
};

#endif