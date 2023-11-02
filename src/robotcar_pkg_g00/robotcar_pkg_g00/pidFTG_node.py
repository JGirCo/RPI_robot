import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class pidFTG_node(Node): 
    def __init__(self):
        super().__init__("pidFTG_node") 
        self.errorActual = 0.0
        self.velocidad_adelante = 1.0
        self.lado = 0.0
        self.kp = 0.0035
        self.kd = 0.0030
        self.der = 0.0
        self.velocidad_angular = 0.5

        # subscriptor obj
        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.error_sub = self.create_subscription(Twist, '/FTGerror', self.error, 1)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_pid_FTG', 10)

        self.timer_period = 0.3 # in [s]
        self.timer = self.create_timer(self.timer_period, self.velocidad)


    def error(self, data):
        self.errorActual = data.linear.x
        self.errorPrev = data.linear.z
    
    def velocidad(self): #el eje de giro z sale del piso
        vel = Twist()
        # vel.linear.x = -0.5 if self.stop_info else 0.5
        vel.linear.x = 1.0
        if(not math.isnan(self.errorActual)):
            vel.angular.z = -(self.kp*self.errorActual + self.kd*self.der/self.timer_period)
        self.get_logger().info('Recibo el error:'+ str(self.errorActual)+'\n\n\n')
        #self.get_logger().info('Vel angular:'+ str(vel.angular.z)+'\n\n\n')
        self.cmd_pub.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = pidFTG_node() # Definicion del objeto "node"

    # ejecucion ciclica 
    rclpy.spin(node)
    # finalizacion
    rclpy.shutdown()

if __name__ == "__main__":
    main()
