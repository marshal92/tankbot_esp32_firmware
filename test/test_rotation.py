import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class RotationTest(Node):
    def __init__(self):
        super().__init__('rotation_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # НАСТРОЙКИ
        self.target_rotations = 5.0   # Сколько полных оборотов сделать
        self.speed_rad_s = 1.5        # Скорость вращения (твоя минимальная)
        
        # Расчет времени: (5 * 2*Pi) / 1.0
        self.duration = (self.target_rotations * 2 * math.pi) / self.speed_rad_s
        
        print(f"STARTING: Rotating {self.target_rotations} times at {self.speed_rad_s} rad/s")
        print(f"Expected Duration: {self.duration:.2f} seconds")
        
        self.timer = self.create_timer(0.1, self.run_test)
        self.start_time = time.time()
        self.running = True

    def run_test(self):
        cmd = Twist()
        elapsed = time.time() - self.start_time

        if elapsed < self.duration:
            cmd.angular.z = self.speed_rad_s
            self.pub.publish(cmd)
            print(f"Turning... {elapsed:.1f}/{self.duration:.1f}s", end='\r')
        else:
            # СТОП
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            self.pub.publish(cmd) # Дублируем для надежности
            print("\nDONE! Stop.")
            self.running = False
            raise SystemExit

rclpy.init()
node = RotationTest()
try:
    rclpy.spin(node)
except SystemExit:
    pass
node.destroy_node()
rclpy.shutdown()