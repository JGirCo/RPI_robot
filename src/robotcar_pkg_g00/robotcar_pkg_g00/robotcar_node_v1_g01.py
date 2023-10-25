#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# ROS python msg libraries
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class RobotCarNodeV1G01(Node): # reemplazar YY por el numero de grupo
    def __init__(self):
        super().__init__("robotcar_node_v1_g01") # Redefine node name
        # def. attributes 
        self.PWM_speed_pin = 13 #channel 1
        self.PWM_steer_pin = 12 #channel 0
        self.speed = 0
        self.steer = 0
        self.envio_speed = 0.0
        self.envio_steer = 0.0

        # topic subscriber
        self.cmd_vel = self.create_subscription(Twist,'/cmd_vel_joy',self.cmd_vel_callback,1)
        self.para_control = self.create_publisher(Twist, '/control', 10)

        self.timer_period = 0.1 # in [s]
        self.timer_period2 = 0.1 # in [s]
        self.timer_period3 = 0.05
        self.timer = self.create_timer(self.timer_period, self.set_motor_speed)
        self.timer2 = self.create_timer(self.timer_period2, self.set_steer)
        self.timer3 = self.create_timer(self.timer_period3, self.enviar)
    
    def cmd_vel_callback(self, msg):
        self.speed = msg.linear.x
        self.steer= msg.angular.z

    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def set_motor_speed(self):
        msg = "speed: {:.3f}, spin: {:.3f}".format(self.speed,self.steer)
        print(msg)

        if self.speed != 0:

            speed_map =	round(self.map(self.speed, -1.0, 1.0, 5.0, 10.0), 1)
            #print('Esto redondeo:' + str(speed_map))
            if 6.9 < speed_map < 7.0: 
                speed_map = 1.0
            elif speed_map >= 7.2:
                speed_map = 7.2
            elif speed_map <= 6.2:
                speed_map = 6.2

            self.envio_speed = speed_map
            
            msg_clip = "----speed: {:.3f}----".format(speed_map)
            print(msg_clip)
        else:
            msg_clip = "----speed: {:.3f}----".format(7.5)
            self.envio_speed = 7.5
            print(msg_clip)

    
    def set_steer(self):
        msg = "speed_steer: {:.3f}, spin_steer: {:.3f}".format(self.speed,self.steer)
        print(msg)

        if self.steer != 0:

            steer_map =	round(self.map(self.steer, -0.5, 0.5, 5.0, 10.0), 1)
            #print('Esto redondeo:' + str(speed_map))
            if 7.3 < steer_map < 7.8: 
                steer_map = 7.5
            # elif steer_map >= 8.0:
            #     steer_map = 8.0
            # elif steer_map <= 6.5:
            #     steer_map = 6.5
            self.envio_steer = steer_map



            msg_clip = "----steer: {:.3f}----".format(steer_map)
            print(msg_clip)
        else:
            msg_clip = "----steer: {:.3f}----".format(7.5)
            print(msg_clip)
            self.envio_steer = 7.5

    def enviar(self):
        envio = Twist()
        envio.linear.x = self.envio_speed
        envio.linear.y = self.envio_steer
        self.para_control.publish(envio)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = RobotCarNodeV1G01() # Definicion del objeto "node"
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