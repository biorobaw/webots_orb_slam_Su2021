import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import numpy as np
from array import array as arr
import random as rand

PI = math.pi
RIGHT_TURN = -PI/2
LEFT_TURN = PI/2
ROBOT_RADIUS = 1.14

class Wanderer(Node):
    def __init__(self):
        super().__init__('wanderer')

        #subscribers
        self.rds_sub = self.create_subscription(Float64, 'rds', self.rds_cb, 1)
        self.lds_sub = self.create_subscription(Float64, 'lds', self.lds_cb, 1)
        self.fds_sub = self.create_subscription(Float64, 'fds', self.fds_cb, 1)
        self.rps_sub = self.create_subscription(Float64, 'rps', self.rps_cb, 1)
        self.lps_sub = self.create_subscription(Float64, 'lps', self.lps_cb, 1)
        self.imu_sub = self.create_subscription(Float64, 'imu', self.imu_cb, 1)
        self.cam_sub = self.create_subscription(Image, 'camera', self.cam_cb, 1)
        
        #publishers
        self.cmdvel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        #parameter initialization
        self.rds, self.lds, self.fds = 0, 0, 0
        self.rps, self.lps = 0, 0
        self.yaw = 0
        self.cmd = Twist()
        self.cam = Image()

        #state machine params
        self.state = 0
        self.direction = None
        self.gYaw = None

    def rds_cb(self, msg):
        self.rds = msg.data
        self.Wander()

    def lds_cb(self, msg):
        self.lds = msg.data

    def fds_cb(self, msg):
        self.fds = msg.data

    def rps_cb(self, msg):
        self.rps = msg.data

    def lps_cb(self, msg):
        self.lps = msg.data

    def imu_cb(self, msg):
        self.yaw = msg.data

    def cam_cb(self, msg):
        self.cam = msg.data

    def getO(self):
        return self.yaw

    def getDistances(self):
        return [self.lds * 39.3701,
                self.rds * 39.3701,
                self.fds * 39.3701]

    #set speeds to turn right
    def turnRight(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = -0.5
        self.cmdvel_publisher.publish(self.cmd)

    #set speeds to turn left
    def turnLeft(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.5
        self.cmdvel_publisher.publish(self.cmd)

    #set speeds to move straight
    def goStraight(self):
        self.cmd.linear.x = 0.5
        self.cmd.angular.z = 0.0
        self.cmdvel_publisher.publish(self.cmd)

    def stop(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.cmdvel_publisher.publish(self.cmd)

    def moveForward(self):
        self.goStraight()
        num = rand.randint(3, 7)
        if(self.getDistances()[2] < num):
            self.stop()
            if self.getDistances()[0] > self.getDistances()[1]:
                self.direction = 'LEFT'
            else:
                self.direction = 'RIGHT'
            self.state = 1

        #implement random turning here


    def turn(self, direction):
        num = rand.randrange(3, 50, 1) / 100
        #turning left
        if direction == 'LEFT': 
            self.turnLeft()  
            #assign goal yaw if not set
            if self.gYaw is None:
                if self.getO() < -1.5 and self.getO() > -1.63: #left
                    self.gYaw = 0
                elif self.getO() < 1.63 and self.getO() > 1.5: #right
                    self.gYaw = PI
                elif (self.getO() < PI + 0.05 and self.getO() > PI - 0.05 or
                    self.getO() > -PI -0.05 and self.getO() < -PI + 0.05): #up
                    self.gYaw = -PI / 2
                elif self.getO() < 0.05 and self.getO() > -0.05: #down
                    self.gYaw = PI / 2
            #just check orientation/yaw to know when to stop turning
            if self.gYaw is not None and self.getO() < self.gYaw + num and self.getO() > self.gYaw - num:
                self.stop()
                self.gYaw = None
                self.direction = None
                self.state = 0
        #urning right
        elif direction == 'RIGHT':   
            self.turnRight()      
            #assign goal yaw if not set
            if self.gYaw is None:
                if self.getO() < -1.5 and self.getO() > -1.63: #left
                    self.gYaw = PI
                elif self.getO() < 1.63 and self.getO() > 1.5: #right
                    self.gYaw = 0
                elif (self.getO() < PI + 0.05 and self.getO() > PI - 0.05 or 
                    self.getO() > -PI -0.05 and self.getO() < -PI + 0.05): #up
                    self.gYaw = PI / 2
                elif self.getO() < 0.05 and self.getO() > -0.05: #down
                    self.gYaw = -PI / 2
            #just check orientation/yaw to know when to stop turning
            if self.gYaw is not None and self.getO() < self.gYaw + num and self.getO() > self.gYaw - num:
                self.stop()
                self.gYaw = None
                self.direction = None
                self.state = 0


    def Wander(self):
        #-----State Machine-----
        #-----------------------
        #--State----Behavior----
        #- 0        Straight
        #- 1        Turn
        if self.state == 0:
            self.moveForward()
            #self.get_logger().info("Going straight...")
        else:
            self.turn(self.direction)
            #self.get_logger().info("Turning...")

        

        #self.get_logger().info("Goal yaw is: {}".format(self.gYaw))
        #self.get_logger().info("Current yaw is: {}".format(self.getO()))


def main(args=None):
    rclpy.init(args=args)

    wm = Wanderer()
    rclpy.spin(wm)

    wm.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()
