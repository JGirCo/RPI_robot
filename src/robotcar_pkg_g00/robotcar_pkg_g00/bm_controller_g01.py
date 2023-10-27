#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import time

# ROS python msg libraries
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from adafruit_servokit import ServoKit

class BMControllerG01(Node):
  
  def __init__(self):
    super().__init__("bm_controller_g01") # Redefine node name
    
    
    self.kit = ServoKit(channels=8)
    self.aux = 0
    self.j_u = 0
    self.j_r = 0
    self.p_u = 0
    self.p_r = 0
    self.pid_u = 0
    self.pid_r = 0
    self.pwd_u = 0
    self.pwd_r = 0
    self.twist_parar = self.create_subscription(Twist,'/cmd_vel_joy',self.send_cmd_joy,10)
    self.twist_parar = self.create_subscription(Twist,'/cmd_parar',self.send_cmd_parar,10)
    self.twist_pid = self.create_subscription(Twist,'/cmd_pid',self.send_cmd_pid,10)

    self.timer_period = 0.1 # in [s]
    self.timer = self.create_timer(self.timer_period, self.send_cmd)
    
  def send_cmd_joy(self,msg):
    #self.get_logger().info('joy: {:.3f}, angular: {:.3f}'.format(msg.linear.x,msg.angular.z))
    self.j_u = msg.linear.x # (-1,1)
    self.j_r = msg.angular.z # (-0.5,0.5)

  def send_cmd_parar(self,msg):
    #self.get_logger().info('Parar: {:.3f}, angular: {:.3f}'.format(msg.linear.x,msg.angular.z))
    self.p_u = msg.linear.x # (-1,1)
    self.p_r  = msg.angular.z # (-0.5,0.5)
  
  def send_cmd_pid(self,msg):
    #self.get_logger().info('pid: {:.3f}, angular: {:.3f}'.format(msg.linear.x,msg.angular.z))
    self.pid_u = msg.linear.x # (-1,1)
    self.pid_r = msg.angular.z # (-0.5,0.5)
  
  def mux(self):
    if self.j_u != 0.0 or self.j_r != 0.0:
      self.pwd_u = self.j_u
      self.pwd_r = self.j_r
    elif self.p_u != 0.0 or self.p_r != 0.0:
      self.pwd_u = self.p_u
      self.pwd_r = self.p_r
    else:
      self.pwd_u = self.pid_u
      self.pwd_r = self.pid_r

  def controlDireccion(self):
    self.kit.continuous_servo[0].set_pulse_width_range(1300, 1700)
    # limiting the speed up to 80%
    if self.pwd_u<0.0:
      #self.get_logger().info('Estoy esperando')
      if self.aux == 0:
        self.kit.continuous_servo[0].throttle = 0.0
        #self.get_logger().info('Quieto: ' + str(0.0))
        time.sleep(0.7)
      self.kit.continuous_servo[0].throttle = (self.pwd_u)*self.kAtras
      #self.get_logger().info('Hacia atras: ' + str((self.pwd_u)*k))
      self.aux = 1
    elif self.pwd_u>0.0:
      self.kit.continuous_servo[0].throttle = (self.pwd_u)*self.kAdelante
      #self.get_logger().info('Hacia adelante: ' + str((self.pwd_u)*k))
      self.aux = 0

    elif self.pwd_u == 0:
      self.kit.continuous_servo[0].throttle = 0.0      
        # range(1200, 2000) equiv (5%, 10%) Servo Steering
        # It can be different for each Car
  def controlMotor(self):
    self.kit.continuous_servo[1].set_pulse_width_range(1300, 2100)
    if (not math.isnan(self.pwd_r) and not math.isinf(self.pwd_r)):
      if (self.pwd_r < -1.0):
        self.kit.continuous_servo[1].throttle = -1.0#self.pwd_r
      elif (self.pwd_r > 1.0):
        self.kit.continuous_servo[1].throttle = 1.0#0.0
      else:
        self.kit.continuous_servo[1].throttle = self.pwd_r
    else:
      self.kit.continuous_servo[1].throttle = 0.0

  def send_cmd(self):
    # range(1300, 1700) equiv (5%, 10%) ESC Speed
    # It can be different for each Car
    self.mux()
    self.kAtras=0.3
    self.kAdelante=0.7
    self.controlDireccion()
    self.controlMotor()
    self.get_logger().info('PWM: linear: {:.3f}, angular: {:.3f}'.format(self.pwd_u, self.pwd_r))

def main(args=None):
  try:
    rclpy.init(args=args)
    node = BMControllerG01() # Definicion del objeto "node"
    print('-----start node------')
    # ejecucion ciclica 
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.destroy_node()
    # finalizacion
    rclpy.shutdown()
    print('------end program-----')


if __name__ == "__main__":
    main()
