import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import numpy as np

def checkForNaN(data):
    if math.isnan(sum(data)):
        return True
    return False

def checkForInf(data):
    if math.isinf(sum(data)):
        return True
    return False

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
        self.distancia_muro = 0.3 # La distancia que buscamos mantener hasta el muro
        self.inicio = 1
        self.lado = 0.0

        # subscriptor obj
        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.scaner, 1)
        
        self.error_pub = self.create_publisher(Twist, '/error', 10)

        timer_period = 0.1 # in [s]
        self.timer = self.create_timer(timer_period, self.error)
    

    

    def getMaxGap(self, distances):
        currentGap = []
        max_gap = []
        for i, distance in enumerate(distances):
            if distance < self.min_dist:
                currentGap = []
                continue
            currentGap.append(i)
            if len(currentGap) <= len(max_gap):
                continue
            max_gap = currentGap
        return max_gap

    def getGapInfo(self, distances, angles,max_gap):
        gapDistances = [distances[i] for i in max_gap]
        gapAngles = [angles[i] for i in max_gap]
        return gapDistances, gapAngles

    def getTargetAngle(self, gapDistances, gapAngles):
        targetAngle = gapAngles[len(gapDistances)//2]
        return targetAngle

    def scaner(self, data):
        ranges = data.ranges
        if checkForNaN(ranges) or checkForInf(ranges):
            self.get_logger().info('Error, inf o nan detectado.' + '\n inf:' + str(checkForInf(ranges)) + '\n' + 'nan:' + str(checkForNaN(ranges)) + '\n\n\n')
            return
        leftRanges = ranges[0:round(len(ranges)/3)]
        rightRanges = ranges[round(2*len(ranges)/3):]
        curatedRanges = (rightRanges + leftRanges)[::-1]
        angles = np.linspace(-120,120, len(ranges))
        maxGap = self.getMaxGap(ranges)
        gapDistances, gapAngles = self.getGapInfo(ranges,angles, maxGap)
        try:
            self.targetAngle = self.getTargetAngle(gapDistances, gapAngles)
        except:
            self.get_logger().info('No gap'+'\n\n\n')


    def scanner(self, data):
        distances = data.ranges.tolist()
        angles = range(self.min_angle,self.max_angle)
        max_gap = self.getMaxGap(distances)
        gapDistances, gapAngles = self.getGapInfo(distances,angles, max_gap)
        try:
            self.targetAngle = self.getTargetAngle(gapDistances, gapAngles)
        except:
            self.get_logger().info('No gap'+'\n\n\n')

        
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
