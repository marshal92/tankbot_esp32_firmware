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

#include "lino_base_config.h"
#include "motor.h"
#include "kinematics.h"
#include "imu.h"
#include "encoder.h"  
#include "pid.h"     

// --- ОБЪЕКТЫ ---
Motor motor1(MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2(MOTOR2_IN_A, MOTOR2_IN_B);

// Энкодеры (reverse = false по умолчанию, если робот считает наоборот - поменяем на true)
Encoder encoder1(MOTOR1_ENC_A, MOTOR1_ENC_B, true); 
Encoder encoder2(MOTOR2_ENC_A, MOTOR2_ENC_B, false);

// PID регуляторы для каждого мотора
PID pid1(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID pid2(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(LINO_BASE, WHEEL_DIAMETER, LR_WHEELS_DISTANCE, MAX_RPM);
IMU imu;

// --- ROS ПЕРЕМЕННЫЕ ---
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg; // cmd_vel

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

// Глобальные переменные для позиции
float pos_x = 0.0;
float pos_y = 0.0;
float theta = 0.0;
unsigned long last_time = 0;

// Целевые скорости
float target_linear_x = 0.0;
float target_angular_z = 0.0;

// Callback скорости
void subscription_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg_in_typed = (const geometry_msgs__msg__Twist *)msgin;
    target_linear_x = msg_in_typed->linear.x;
    target_angular_z = msg_in_typed->angular.z;
}

// Callback фары
void light_callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    // Включаем или выключаем пин (true/false)
    digitalWrite(LIGHT_PIN, msg->data ? HIGH : LOW);
}

void setup() {
 Serial.begin(921600);
  set_microros_serial_transports(Serial);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  digitalWrite(LIGHT_PIN, LOW);

  // Синхронизация
    rmw_uros_sync_session(1000); 

  // Инициализация железа
    encoder1.init();
    encoder2.init();

  if (imu.init()) {
      digitalWrite(LED_PIN, HIGH); 
  }

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "linorobot_base", "", &support);

  // Подписчик CMD_VEL
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel");
  
  // Подписчик cmd_light
  rclc_subscription_init_default(
    &light_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "cmd_light");

  // Паблишеры
  rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data");

  rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom/unfiltered");

  // Настройка ковариации
  // ODOM
  odom_msg.header.frame_id.data = (char*)"odom";
  odom_msg.header.frame_id.size = strlen("odom");
  odom_msg.child_frame_id.data = (char*)"base_link";
  odom_msg.child_frame_id.size = strlen("base_link");
  odom_msg.pose.covariance[0] = 0.00001;
  odom_msg.pose.covariance[7] = 0.00001;
  odom_msg.pose.covariance[35] = 0.001;

  // IMU
  imu_msg.header.frame_id.data = (char*)"imu_link";
  imu_msg.header.frame_id.size = strlen("imu_link");
  imu_msg.orientation_covariance[0] = 0.1; // Ориентация из IMU (если есть)
  imu_msg.angular_velocity_covariance[0] = 0.0001;
  imu_msg.linear_acceleration_covariance[0] = 0.0001;

  // ЭКЗЕКУТОР
  rclc_executor_init(&executor, &support.context, 2, &allocator);
 
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &light_subscriber, &light_msg, &light_callback, ON_NEW_DATA);

  last_time = millis();
}

void loop() {
 
  // Проверяем связь с Агентом (Пинг)
  static unsigned long last_ping = 0;
  if (millis() - last_ping > 1000) {
      last_ping = millis();
      if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
          // Если мы потеряли комп - перезагружаемся!
          motor1.spin(0);
          motor2.spin(0);
          ESP.restart(); 
      }
  }
 
  // Синхронизация времени раз в минуту
  if (millis() % 10000 == 0) {
      rmw_uros_sync_session(100);
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  // // === ФИЗИКА И УПРАВЛЕНИЕ ===
    unsigned long now = millis();
    float dt = (now - last_time) / 1000.0;
    
    // Выполняем расчет только если прошло достаточно времени
    if (dt >= 0.05) { // 20 Гц
        last_time = now;

        // 1. Читаем энкодеры
        long enc1_ticks = encoder1.read();
        long enc2_ticks = encoder2.read();
        
        // Для PID проще сбросить и знать, сколько набежало за dt.
        encoder1.write(0);
        encoder2.write(0);

        // 2. Расчет RPM
        float current_rpm1 = ((float)enc1_ticks / COUNTS_PER_REV) * (60.0 / dt);
        float current_rpm2 = ((float)enc2_ticks / COUNTS_PER_REV) * (60.0 / dt);

        // 3. Одометрия
        velocities current_vel = kinematics.getVelocities(current_rpm1, current_rpm2);

        // Интегрируем позицию
        float delta_th = current_vel.angular_z * dt;
        float delta_x = (current_vel.linear_x * cos(theta)) * dt;
        float delta_y = (current_vel.linear_x * sin(theta)) * dt;

        pos_x += delta_x;
        pos_y += delta_y;
        theta += delta_th;

        // 4. Фильтрация шума команд(Hard Limits для плавности)
        // Создаем локальные копии команд, чтобы применить фильтр
        float cmd_lin = target_linear_x;
        float cmd_ang = target_angular_z;

        // ЛИНЕЙНАЯ СКОРОСТЬ
        if (abs(cmd_lin) > 0.001 && abs(cmd_lin) < MIN_ABS_LIN) {
            cmd_lin = (cmd_lin > 0) ? MIN_ABS_LIN : -MIN_ABS_LIN;
        }

        // УГЛОВАЯ СКОРОСТЬ
        if (abs(cmd_ang) > 0.001 && abs(cmd_ang) < MIN_ABS_ANG) {
            cmd_ang = (cmd_ang > 0) ? MIN_ABS_ANG : -MIN_ABS_ANG;
        }

        // 5. Расчет PID и Моторов
        // Получаем целевые RPM из Kinematics (на основе cmd_vel)
        rpm target_rpm = kinematics.getRPM(cmd_lin, 0.0, cmd_ang);
        
        // Если цель 0, сбрасываем PID (чтобы не накапливалась ошибка)
        if (target_linear_x == 0 && target_angular_z == 0) {
             pid1.reset();
             pid2.reset();
             motor1.spin(0);
             motor2.spin(0);
        } else {
             // Считаем PID
             int pwm1 = pid1.compute(target_rpm.motor1, current_rpm1);
             int pwm2 = pid2.compute(target_rpm.motor2, current_rpm2);
             
             motor1.spin(pwm1);
             motor2.spin(pwm2);
        }

        // 6. Публикация данных (ROS2)
        struct timespec tv = {0};
        clock_gettime(CLOCK_REALTIME, &tv);

        // ODOM MSG
        odom_msg.header.stamp.sec = tv.tv_sec;
        odom_msg.header.stamp.nanosec = tv.tv_nsec;
        
        odom_msg.pose.pose.position.x = pos_x;
        odom_msg.pose.pose.position.y = pos_y;
        odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
        odom_msg.pose.pose.orientation.w = cos(theta / 2.0);
        
        // В twist пишем РЕАЛЬНУЮ скорость (feedback), а не команду
        odom_msg.twist.twist.linear.x = current_vel.linear_x;
        odom_msg.twist.twist.angular.z = current_vel.angular_z;

        // IMU MSG
        imu_msg.header.stamp.sec = tv.tv_sec;
        imu_msg.header.stamp.nanosec = tv.tv_nsec;
        imu_msg.linear_acceleration = imu.readAccelerometer();
        imu_msg.angular_velocity = imu.readGyroscope();

        rcl_publish(&imu_publisher, &imu_msg, NULL);
        rcl_publish(&odom_publisher, &odom_msg, NULL);
    }
}