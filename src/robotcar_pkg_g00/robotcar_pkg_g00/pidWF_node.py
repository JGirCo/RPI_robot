import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class pidWF_node(Node): 
    def __init__(self):
        super().__init__("pidWF_node") 
        self.error_actual = 0.0
        self.velocidad_adelante = 1.0
        self.lado = 0.0
        self.kp = 2.0
        self.kd = 2.0
        self.der = 0.0
        self.velocidad_angular = 0.5

        # subscriptor obj
        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.error_sub = self.create_subscription(Twist, '/error', self.error, 1)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_pid', 10)

        self.timer_period = 0.3 # in [s]
        self.timer = self.create_timer(self.timer_period, self.velocidad)

    def error(self, data):
        self.error_actual = data.linear.x
        self.lado = data.linear.y
        self.der = data.linear.z #lo voy a usar para la derivada del error
        self.cd = data.angular.x
        self.cd2 = data.angular.y
    
    def stop(self):
        self.vel = Twist()
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.cmd_pub.publish(self.vel)

    def setVel(self):
        self.vel.angular.z = self.kp*self.error_actual + self.kd*self.der/self.timer_period
        if(self.lado == 0.0):
            self.vel.angular.z = -self.vel.angular.z
    def cutoffVel(self):
        maxAngle = 0.5
        self.vel.angular.z = min(max(self.vel.angular.z, -maxAngle),maxAngle)

        
    def velocidad(self): #el eje de giro z sale del piso
        self.vel = Twist()
        self.vel.linear.x = self.velocidad_adelante
        if(math.isnan(self.error_actual) or math.isinf(self.error_actual)):
            self.stop()
            return
        self.setVel()
        self.cutoffVel()


        #self.get_logger().info('Recibo el error:'+ str(self.error_mio)+'\n\n\n')
        #self.get_logger().info('Vel lineal:'+ str(self.vel.linear.x)+'\n\n')
        #self.get_logger().info('Vel angular:'+ str(self.vel.angular.z)+'\n\n\n')
        self.cmd_pub.publish(self.vel)

def main(args=None):
    rclpy.init(args=args)
    node = pidWF_node() # Definicion del objeto "node"

    # ejecucion ciclica 
    rclpy.spin(node)
    # finalizacion
    rclpy.shutdown()

if __name__ == "__main__":
    main()
