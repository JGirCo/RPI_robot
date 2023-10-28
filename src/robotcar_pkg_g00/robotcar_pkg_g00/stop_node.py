import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class stop_node(Node): 
    def __init__(self):
        super().__init__("stop_node") 
        self.ranges = [0.0]
        self.distancia_segura = 0.5
        self.velocidad_angular = 0.5
        self.lado = 0.0

        # subscriptor obj
        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.scaner, 1)
        self.error_sub = self.create_subscription(Twist, '/error', self.error, 1)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_parar', 10)

        timer_period = 0.1 # in [s]
        self.timer = self.create_timer(timer_period, self.parar)

    def scaner(self, data):
        self.ranges = data.ranges
        #self.get_logger().info('rango 1:'+ str(self.ranges[0])+'\n\n\n')
        #self.get_logger().info('rango 2:'+ str(self.ranges[179])+'\n\n\n')

    def error(self, data):
        self.lado = data.linear.y

    def parar(self):
        vel = Twist()
        n_60 = round(11*len(self.ranges)/12) - 1 # 330, nuevo 60
        n_120 = round(1*len(self.ranges)/12) - 1 # 30, nuevo 120

        #self.get_logger().info('ranges.n_60: '+ str(self.ranges[n_60]) +' ranges.n_120: '+str(self.ranges[n_120])+'\n')
        #self.get_logger().info('n_60: '+ str(n_60) +' n_120: '+str(n_120)+'\n')
        #self.get_logger().info('len.ranges: ' + str(len(self.ranges)) + '\n')

        vel.linear.x = 0.0
        vel.angular.z = 0.0

        rangos = list(range(n_60, len(self.ranges)-1)) + list(range(1, n_120))
        #self.get_logger().info('rangos: ' + str(rangos) + '\n')
        
        for i in rangos:
            if(math.isnan(self.ranges[i]) or math.isinf(self.ranges[i])):
                continue
            if not (self.ranges[i] <= self.distancia_segura and (self.ranges.index(self.ranges[i]) > n_60 or self.ranges.index(self.ranges[i]) < n_120)):
                continue
            if (self.lado == 0.0):
                vel.angular.z = self.velocidad_angular
            else:
                vel.angular.z = -self.velocidad_angular
            vel.linear.x = -1.0
            #self.get_logger().info('minima distancia:'+ str(min(self.ranges))+'indice: '+str(self.ranges.index(min(self.ranges)))+'\n')
            #self.get_logger().info('Pa lado:'+ str(vel.angular.z)+'\n\n\n')
        #self.get_logger().info('minima distancia:'+ str(min(self.ranges))+'indice: '+str(self.ranges.index(min(self.ranges)))+'\n')

        self.cmd_pub.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = stop_node() # Definicion del objeto "node"

    # ejecucion ciclica 
    rclpy.spin(node)
    # finalizacion
    rclpy.shutdown()

if __name__ == "__main__":
    main()
