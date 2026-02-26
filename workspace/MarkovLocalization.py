#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64MultiArray

class MarkovLocalization(Node):

    # Inicializando nó
    def __init__(self):
        super().__init__('markov_localization')
        self.get_logger().info('Inicializando o nó!')

        self.robot_front_laser = None
        self.robot_side_laser = None
        self.subscription = self.create_subscription(LaserScan, '/scan', self.subscriber_callback, 10)

        self.publisher_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_bel = self.create_publisher(Float64MultiArray, '/belief', 10)
       
        self.timer_period = 1.0
        self.timer = self.create_timer(self.timer_period,self.timer_callback)
        rclpy.spin(self)

    # Finalizando nó
    def __del__(self):
        self.get_logger().info('Finalizando o nó!')
        self.destroy_node()

    def subscriber_callback(self, msg):
        self.robot_front_laser = msg.ranges[0]
        self.robot_side_laser = msg.ranges[90]

    def timer_callback(self):
        msg_stop = Twist(linear=Vector3(x=0.0,y=0.0,z=0.0), angular=Vector3(x=0.0,y=0.0,z=0.0))
        msg_forw = Twist(linear=Vector3(x=0.2,y=0.0,z=0.0), angular=Vector3(x=0.0,y=0.0,z=0.0))

        self.publisher_vel.publish(msg_stop)

        self.get_logger().info(f'Valor do laser lateral do robô: {self.robot_side_laser}')

        if(self.robot_front_laser > 1.0):
            self.publisher_vel.publish(msg_forw)


def main(args=None):
    rclpy.init(args=args) # Inicializando ROS
    node = MarkovLocalization() # Inicializando nó
    del node              # Finalizando nó
    rclpy.shutdown()      # Finalizando ROS

if __name__ == '__main__':
    main()
