#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial
import os

class BMControllerG00(Node):
    VERSION = '0.1'
    SERIAL_PORT = '/dev/ttyS0'
    # Update based on actual base specs.
    PWD_MIN = 10 # Minumum PWD supported by base
    LINEAR_TO_PWD = 100 # if linear.x = 2 m/s then set speed (pwd) to 100
    ANGULAR_TO_PWD = 100 # if linear.x = 2 m/s then set speed (pwd) to 100

    def __init__(self):
        super().__init__("bm_controller_g00") # Redefine node name

        if not os.path.exists(self.SERIAL_PORT):
            self.get_logger().error("Serial Port not found:" + self.SERIAL_PORT + " bm_controller not started")
            rclpy.shutdown()

        self.ser = serial.Serial(self.SERIAL_PORT,115200, parity=serial.PARITY_NONE,  stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS,timeout=0)  # open serial port without blocking
        self.u_msg = ""
        self.r_msg = ""
        self.send_serial() # Send to clear out any noise in the serial buffer 

        self.u_msg = "Hola mundo"
        self.r_msg = "Hello world"

        # Fire up an asyncronous timer to check for messages from the PICO on SERIAL_PORT
        #self.serial_read_timer = self.create_timer(0.1 , self.read_serial) 

        self.serial_write_timer = self.create_timer(0.1, self.send_serial) 
        self.base_movement_info = ""

        self.get_logger().info("bm_controller has started: " +  self.VERSION)

    def send_serial(self):
        self.get_logger().info("sending serial:" + self.u_msg)
        self.ser.write((self.u_msg  + "\n").encode())
        self.get_logger().info("sending serial:" + self.r_msg)
        self.ser.write((self.r_msg  + "\n").encode())

    def read_serial(self):
        pass

    def process_pose(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = BMControllerG00() # Definicion del objeto "node"

    # ejecucion ciclica 
    rclpy.spin(node)
    # finalizacion
    rclpy.shutdown()


if __name__ == '__main__':
    main()
