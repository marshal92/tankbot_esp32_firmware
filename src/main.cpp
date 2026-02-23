#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h> 
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/bool.h>
#include <rmw_microros/rmw_microros.h>
#include <Wire.h>

#include "lino_base_config.h"
#include "motor.h"
#include "kinematics.h"
#include "imu.h"
#include "encoder.h"  
#include "pid.h"     
#include "Ramp.h"

#define SERIAL_BAUD 115200      
#define CMD_INTERVAL_MS 20      
#define LOOP_INTERVAL_S 0.01    
#define WATCHDOG_TIMEOUT 500    

#define LINEAR_ACCEL 0.02  
#define ANGULAR_ACCEL 0.06

Motor motor1(MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2(MOTOR2_IN_A, MOTOR2_IN_B);
Encoder encoder1(MOTOR1_ENC_A, MOTOR1_ENC_B, true); 
Encoder encoder2(MOTOR2_ENC_A, MOTOR2_ENC_B, false);
PID pid1(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID pid2(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
Kinematics kinematics(LINO_BASE, WHEEL_DIAMETER, LR_WHEELS_DISTANCE, MAX_RPM);
IMU imu;

Ramp ramp_linear(LINEAR_ACCEL);
Ramp ramp_angular(ANGULAR_ACCEL);

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rcl_subscription_t light_subscriber;
std_msgs__msg__Bool light_msg;
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;
rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

float pos_x = 0.0; float pos_y = 0.0; float theta = 0.0;
unsigned long last_time = 0;

float target_linear_x_raw = 0.0; 
float target_angular_z_raw = 0.0;
unsigned long last_cmd_received_time = 0; 

void subscription_callback(const void * msgin) {
    last_cmd_received_time = millis(); 
    static unsigned long last_processed_cmd = 0;
    if (millis() - last_processed_cmd < CMD_INTERVAL_MS) return; 
    last_processed_cmd = millis();

    const geometry_msgs__msg__Twist * msg_in_typed = (const geometry_msgs__msg__Twist *)msgin;
    
    target_linear_x_raw = msg_in_typed->linear.x;
    target_angular_z_raw = msg_in_typed->angular.z;
}

void light_callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    digitalWrite(LIGHT_PIN, msg->data ? HIGH : LOW);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  set_microros_serial_transports(Serial);
  
  // --- ВАЖНО: Назначаем I2C пины для S3 ---
  Wire.begin(I2C_SDA, I2C_SCL);

  pinMode(LED_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  digitalWrite(LIGHT_PIN, LOW); 

  encoder1.init();
  encoder2.init();

  ramp_linear.reset();
  ramp_angular.reset();

  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(100);
  }
  digitalWrite(LED_PIN, LOW); 

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "linorobot_base", "", &support);
  
  rmw_uros_sync_session(1000);

  if (imu.init()) {
      for(int i=0; i<3; i++) { digitalWrite(LIGHT_PIN, HIGH); delay(50); digitalWrite(LIGHT_PIN, LOW); delay(50); }
  }

  rclc_subscription_init_best_effort(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
  rclc_subscription_init_default(&light_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "cmd_light");

  //rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw");
  //rclc_publisher_init_default(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom/unfiltered");
  rclc_publisher_init_best_effort(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw");
  rclc_publisher_init_best_effort(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom/unfiltered");

  odom_msg.header.frame_id.data = (char*)"odom";
  odom_msg.header.frame_id.size = strlen("odom");
  odom_msg.child_frame_id.data = (char*)"base_link";
  odom_msg.child_frame_id.size = strlen("base_link");
  odom_msg.pose.covariance[0] = 0.0001; 
  odom_msg.pose.covariance[7] = 0.0001; 
  odom_msg.pose.covariance[35] = 0.1;
  odom_msg.pose.pose.orientation.w = 1.0; 

  imu_msg.header.frame_id.data = (char*)"imu_link";
  imu_msg.header.frame_id.size = strlen("imu_link");
  imu_msg.orientation_covariance[0] = 0.1; 
  imu_msg.orientation_covariance[4] = 0.1;
  imu_msg.orientation_covariance[8] = 0.1;

  imu_msg.angular_velocity_covariance[0] = 0.02; 
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[8] = 0.02;

  imu_msg.linear_acceleration_covariance[0] = 0.4;
  imu_msg.linear_acceleration_covariance[4] = 0.4;
  imu_msg.linear_acceleration_covariance[8] = 0.4;

  imu_msg.orientation.w = 1.0; 

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &light_subscriber, &light_msg, &light_callback, ON_NEW_DATA);
  
  last_time = millis();
  last_cmd_received_time = millis(); 
}

void loop() {

  if (rmw_uros_epoch_millis() < 10000000 || millis() % 10000 < 50) { 
      rmw_uros_sync_session(20); 
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0)); 

  unsigned long now = millis();
  
  if ((now - last_cmd_received_time) > WATCHDOG_TIMEOUT) {
      target_linear_x_raw = 0.0; target_angular_z_raw = 0.0;
  }

  float dt = (now - last_time) / 1000.0;
  if (dt == 0) return;

  if (dt >= LOOP_INTERVAL_S) { 
      last_time = now;
      
      long enc1 = encoder1.read(); long enc2 = encoder2.read();
      encoder1.write(0); encoder2.write(0);

      float rpm1 = ((float)enc1 / COUNTS_PER_REV) * (60.0 / dt);
      float rpm2 = ((float)enc2 / COUNTS_PER_REV) * (60.0 / dt);

      velocities vel = kinematics.getVelocities(rpm1, rpm2);

      float d_th = vel.angular_z * dt;
      float d_x = (vel.linear_x * cos(theta)) * dt;
      float d_y = (vel.linear_x * sin(theta)) * dt;
      pos_x += d_x; pos_y += d_y; theta += d_th;

      if (isnan(pos_x) || isnan(pos_y) || isnan(theta)) { pos_x=0; pos_y=0; theta=0; vel.linear_x=0; vel.angular_z=0; }

      if (target_linear_x_raw == 0) {
          ramp_linear.setStep(0.9); // 1.0 = мгновенно
      } else {
          ramp_linear.setStep(0.02); // 0.02 = мягкий старт
      }
      float smooth_linear = ramp_linear.update(target_linear_x_raw);

      if (target_angular_z_raw == 0) {
          ramp_angular.setStep(1.0); 
      } else {
          ramp_angular.setStep(0.06); 
      }
      float smooth_angular = ramp_angular.update(target_angular_z_raw);

      if (abs(smooth_linear) < 0.02) smooth_linear = 0;
      if (abs(smooth_angular) < 0.02) smooth_angular = 0;

      rpm t_rpm = kinematics.getRPM(smooth_linear, 0.0, smooth_angular);

      if (smooth_linear == 0 && smooth_angular == 0) {
           //pid1.reset(); pid2.reset();
           motor1.spin(0); motor2.spin(0);
      } else {
           motor1.spin(pid1.compute(t_rpm.motor1, rpm1));
           motor2.spin(pid2.compute(t_rpm.motor2, rpm2));
           
           // ВРЕМЕННЫЙ ВЫВОД RPM (можно раскомментировать для отладки)
           //int test_pwm = 50; 
           //motor1.spin(t_rpm.motor1 > 0 ? test_pwm : (t_rpm.motor1 < 0 ? -test_pwm : 0));
           //motor2.spin(t_rpm.motor2 > 0 ? test_pwm : (t_rpm.motor2 < 0 ? -test_pwm : 0));
      }

      int64_t time_ns = rmw_uros_epoch_nanos();
      
      odom_msg.header.stamp.sec = time_ns / 1000000000;
      odom_msg.header.stamp.nanosec = time_ns % 1000000000;
      
      odom_msg.pose.pose.position.x = pos_x;
      odom_msg.pose.pose.position.y = pos_y;
      odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
      odom_msg.pose.pose.orientation.w = cos(theta / 2.0);
      if (isnan(odom_msg.pose.pose.orientation.z)) { odom_msg.pose.pose.orientation.z = 0.0; odom_msg.pose.pose.orientation.w = 1.0; }
      odom_msg.twist.twist.linear.x = vel.linear_x;
      odom_msg.twist.twist.angular.z = vel.angular_z;

      // --- ВРЕМЕННЫЙ ВЫВОД RPM ---
      // Прячем обороты левого (y) и правого (z) колеса в неиспользуемые оси
      //odom_msg.twist.twist.linear.y = rpm1; 
      //odom_msg.twist.twist.linear.z = rpm2; 
      // ---------------------------

      imu_msg.header.stamp = odom_msg.header.stamp;
      imu_msg.linear_acceleration = imu.readAccelerometer();
      imu_msg.angular_velocity = imu.readGyroscope();
      imu_msg.orientation.x = 0.0; imu_msg.orientation.y = 0.0; imu_msg.orientation.z = 0.0; imu_msg.orientation.w = 1.0; 

      (void)rcl_publish(&imu_publisher, &imu_msg, NULL);
      (void)rcl_publish(&odom_publisher, &odom_msg, NULL);
  }
}