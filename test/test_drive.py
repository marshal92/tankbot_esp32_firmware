import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class LinearTest(Node):
    def __init__(self):
        super().__init__('linear_test_node')
        self.target_dist = 2.0  # ЕХАТЬ РОВНО 2 МЕТРА
        self.speed = 0.4        # Скорость м/с
        
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom/unfiltered', self.odom_cb, 10)
        
        self.start_x = None
        self.moved_dist = 0
        self.moving = False
        print("Waiting for Odom...")

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.moving = True
            print("STARTING DRIVE!")
            return

        if self.moving:
            # Считаем, сколько проехали
            dx = x - self.start_x
            dy = y - self.start_y
            self.moved_dist = math.sqrt(dx*dx + dy*dy)
            
            cmd = Twist()
            if self.moved_dist < self.target_dist:
                cmd.linear.x = self.speed
                print(f"Dist: {self.moved_dist:.3f} / {self.target_dist} m", end='\r')
            else:
                cmd.linear.x = 0.0
                self.pub.publish(cmd)
                print(f"\nDONE! Odom thinks: {self.moved_dist:.4f} m")
                self.moving = False
                raise SystemExit
            
            self.pub.publish(cmd)

rclpy.init()
node = LinearTest()
try: rclpy.spin(node)
except SystemExit: pass
node.destroy_node()
rclpy.shutdown()