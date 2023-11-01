import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class laser_node(Node): 
    def __init__(self):
        super().__init__("laser_node") 

        # Todo lo que tenga 2 es de la izquierda los otros de la derecha

        self.ac = 0.1 # Es la distancia que supongo que se mueve el carro (para adelante) 
                    # antes de calcular su distancia hacia la pared
        self.l = 0.1 # Distancia que supongo que se mueve el carro para el calculo del 
                    # angulo para mantenerlo paralelo al muro, en realidad no es un valor
                    # importante ya que al final lo que buscamos es que theta sea 0 
        self.cd = 0.0
        self.cd_2 = 0.0 
        self.theta = 0.0 # Queremos que sea 0, pues al estar en ese valor se asegura que el carro este paralelo al muro
        self.y = 0.0 # Queremos que sea 0, ya que es el error de la distancia al muro
        self.anterior = 0.0
        self.distancia_muro = 0.5 # La distancia que buscamos mantener hasta el muro
        self.inicio = 1
        self.lado = 0.0

        # subscriptor obj
        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.scaner, 1)
        
        self.error_pub = self.create_publisher(Twist, '/error', 10)

        timer_period = 0.1 # in [s]
        self.timer = self.create_timer(timer_period, self.error)
    
    def scaner(self, data):
        ranges = data.ranges
        b = ranges[round(3*len(ranges)/4) - 1] # 270, nuevo 0
        a = ranges[round(5*len(ranges)/6) - 1] # 300, nuevo 30 
        b_2 = ranges[round(len(ranges)/4) - 1] # 90, nuevo 180
        a_2 = ranges[round(1*len(ranges)/6) - 1] # 60, nuevo 150
        #self.get_logger().info('nuevo 0: ' + str(b) + '\n' + 'nuevo 30: ' + str(a) + '\n' + 'nuevo 180: ' + str(b_2) + '\n' + 'nuevo 150: ' + str(a_2) +  '\n\n\n')
        alfa = math.atan((a*math.cos(math.pi/6)-b)/(a*math.sin(math.pi/6)))
        alfa_2 = math.atan((a_2*math.cos(math.pi/6)-b_2)/(a_2*math.sin(math.pi/6)))
        ab = b*math.cos(alfa)
        ab_2 = b_2*math.cos(alfa_2)
        self.cd = ab + self.ac*math.sin(alfa)
        self.cd_2 = ab_2 + self.ac*math.sin(alfa_2)
        #self.get_logger().info('minima distancia:'+ str(min(ranges))+'indice: '+str(ranges.index(min(ranges)))+'\n')

        if self.inicio == 1:
            if self.cd <= self.cd_2:
                self.theta = -alfa 
                self.y = self.distancia_muro - self.cd
                self.inicio = 0 
                self.lado = 0.0
            else:
                self.theta = -alfa_2 
                self.y = self.distancia_muro - self.cd_2
                self.inicio = 2
                self.lado = 1.0
        elif self.inicio == 0:
            self.theta = -alfa 
            self.y = self.distancia_muro - self.cd
        elif self.inicio == 2:
            self.theta = -alfa_2 
            self.y = self.distancia_muro - self.cd_2
        
        #self.get_logger().info('Longitud del arreglo:' + str(len(ranges)) + '\n' + '270° (nuevo 0°):' + str(ranges[round(3*len(ranges)/4) - 1]) + '\n' + '90° (nuevo 180°):' + str(ranges[round(len(ranges)/4) - 1]) + '\n\n\n')
        #self.get_logger().info('cd:' + str(self.cd) + '\n' + 'cd_2:' + str(self.cd_2) + '\n\n\n')


    def error(self):
        error = Twist()
        # error.linear.x es el error total actual
        # error.linear.y es la variable que uso para saber cual muro esta siguiendo
        # error.linear.z es la diferencia de errores, el acutal menos el anterior

        error.linear.y = self.lado
        
        error.linear.x = -(self.y + self.l*math.sin(self.theta))
        error.linear.z = error.linear.x - self.anterior
        self.anterior = error.linear.x
        error.angular.x = self.cd
        error.angular.y = self.cd_2
        #self.get_logger().info('Envio el error:'+ str(error.position.x)+'\n\n\n')

        self.error_pub.publish(error)

def main(args=None):
    rclpy.init(args=args)
    node = laser_node() # Definicion del objeto "node"

    # ejecucion ciclica 
    rclpy.spin(node)
    # finalizacion
    rclpy.shutdown()

if __name__ == "__main__":
    main()