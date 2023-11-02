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

class FTG_node(Node): 
    def __init__(self):
        super().__init__("FTG_node") 
        self.min_dist = 2
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.scaner, 1)
        
        self.error_pub = self.create_publisher(Twist, '/FTGerror', 10)

        timer_period = 0.1 # in [s]
        self.timer = self.create_timer(timer_period, self.error)
        self.prevError = 0.0
    

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
        if checkForNaN(ranges):
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

        
        #self.get_logger().info('Longitud del arreglo:' + str(len(ranges)) + '\n' + '270° (nuevo 0°):' + str(ranges[round(3*len(ranges)/4) - 1]) + '\n' + '90° (nuevo 180°):' + str(ranges[round(len(ranges)/4) - 1]) + '\n\n\n')
        #self.get_logger().info('cd:' + str(self.cd) + '\n' + 'cd_2:' + str(self.cd_2) + '\n\n\n')


    def error(self):
        error = Twist()
        # error.linear.x es el error total actual
        # error.linear.z es la diferencia de errores, el acutal menos el anterior
        self.error_pub.publish(error)
        error = Twist()
        error.linear.x = self.targetAngle
        error.linear.z = self.prevError

        self.prevError = self.targetAngle
        self.error_pub.publish(error)

def main(args=None):
    rclpy.init(args=args)
    node = FTG_node() # Definicion del objeto "node"

    # ejecucion ciclica 
    rclpy.spin(node)
    # finalizacion
    rclpy.shutdown()

if __name__ == "__main__":
    main()
