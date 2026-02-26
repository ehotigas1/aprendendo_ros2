#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64 

import numpy as np  

class KalmanFilter(Node):

    # Inicializando nó
    def __init__(self):
        super().__init__('kalman_filter')
        self.get_logger().info('Inicializando o nó!')

        self.create_subscription(Odometry, '/odom', self.subscriber_callback, 10)
        self.publisher_x = self.create_publisher(Float64, '/x', 10)

        ########## inicializando variáveis ##########
        self.dT = 1.0                                  # variacao do tempo
        self.x = np.array([[0.0],[0.0]])                # vetor do estado
        self.P = np.identity(2)*10e-5                   # matriz de covariância da estimativa inicial
        
        self.z = None                                   # medição em cada passo
        self.u = None                                   # vetor de controle
        self.Q = np.array([[0.1**2,0],[0,0.1**2]])      # covariância do ruído de processo
        self.R = None                                   # covariância do ruído de medição
        #############################################

        self.timer = self.create_timer(self.dT,self.timer_callback)
        rclpy.spin(self)

    # Finalizando nó
    def __del__(self):
        self.get_logger().info('Finalizando o nó!')
        self.destroy_node()

    # Odom callback
    def subscriber_callback(self, msg):
        self.z = np.array([[msg.pose.pose.position.x]])
        self.u = np.array([[0]])
        self.R = np.diag([msg.pose.covariance[0]])

    # Executando nó
    def timer_callback(self):
        if(self.z != None or self.u != None):
            self.x, self.P = self.kf_algorithm(self.x,self.P,self.u,self.z,self.Q,self.R)

        msg = Float64()
        msg.data = self.x[0,0]

        self.publisher_x.publish(msg)

    # algorithm
    def kf_algorithm(self, x, P, u, z, Q, R):
        A = np.array([[1, self.dT],[0, 1]]) # matriz de transição de estado
        B = np.array([[0.5*self.dT**2],[self.dT]])              # matriz de controle
        H = np.array([[1, 0]])              # matriz de observação

        I = np.identity(2)                   # matriz identidade

        ## implementar algoritmo aqui! ##


        # predição
        x_pred = A @ x + B @ u
        P_pred = A @ P @ A.T + Q

        # correção
        Kt = P_pred @ H.T @ np.linalg.inv(H @ P_pred @ H.T + R)
        x = x_pred + Kt @ (z - H @ x_pred)
        P = (I - Kt @ H) @ P_pred
        
        #################################

        # resultado
        print("Estado coletado:\n", x_pred)
        print("Estado corrigido:\n", x)

        return x.copy(), P.copy()


def main(args=None):
    rclpy.init(args=args) # Inicializando ROS
    node = KalmanFilter() # Inicializando nó
    del node              # Finalizando nó
    rclpy.shutdown()      # Finalizando ROS

if __name__ == '__main__':
    main()
