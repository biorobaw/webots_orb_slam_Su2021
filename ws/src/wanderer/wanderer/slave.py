import rclpy
from tf2_ros import TFMessage, TransformStamped
from webots_ros2_core.webots_node import WebotsNode
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image, CameraInfo
from rclpy.time import Time
from tf2_ros import StaticTransformBroadcaster
from sensor_msgs.msg import LaserScan, CompressedImage
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Transform
from webots_ros2_core.math.interpolation import interpolate_lookup_table
from webots_ros2_core.webots_differential_drive_node import WebotsDifferentialDriveNode


from math import pi
import time as time

DEFAULT_WHEEL_RADIUS = 0.02
DEFAULT_WHEEL_DISTANCE = 0.05685

class ServiceNodeVelocity(WebotsDifferentialDriveNode):
    def __init__(self, args):
        super().__init__('slave_node', args,
            wheel_distance=DEFAULT_WHEEL_DISTANCE,
            wheel_radius=DEFAULT_WHEEL_RADIUS)

        self.timestep = 16
        self.camera_timestep = self.timestep

        #Sensor section
        self.sensor_timer = self.create_timer(
            self.timestep / 10000, self.sensor_callback
        )

        #distance sensor
        self.rds = self.robot.getDevice('right_ds')
        self.rds.enable(self.timestep)
        self.rds_publisher = self.create_publisher(Float64, 'rds', 1)

        self.fds = self.robot.getDevice('front_ds')
        self.fds.enable(self.timestep)
        self.fds_publisher = self.create_publisher(Float64, 'fds', 1)

        self.lds = self.robot.getDevice('left_ds')
        self.lds.enable(self.timestep)
        self.lds_publisher = self.create_publisher(Float64, 'lds', 1)

        self.get_logger().info('Enabled Distance Sensors')

        #position sensors
        self.rps = self.robot.getDevice('right wheel sensor')
        self.rps.enable(self.timestep)
        self.rps_publisher = self.create_publisher(Float64, 'rps', 1)

        self.lps = self.robot.getDevice('left wheel sensor')
        self.lps.enable(self.timestep)
        self.lps_publisher = self.create_publisher(Float64, 'lps', 1)

        self.get_logger().info('Enabled Position Sensors')

        #wheels
        self.lm = self.robot.getDevice('left wheel motor')
        self.lm.setPosition(float('inf'))
        self.lm.setVelocity(0)

        self.rm = self.robot.getDevice('right wheel motor')
        self.rm.setPosition(float('inf'))
        self.rm.setVelocity(0)

        self.get_logger().info('Enabled wheel motors')

        #imu
        self.imu = self.robot.getDevice('inertial unit')
        self.imu.enable(self.timestep)
        self.imu_publisher = self.create_publisher(Float64, 'imu', 1)

        self.get_logger().info('Enabled IMU')

        #camera
        self.camera = self.robot.getDevice('camera1')
        self.camera.enable(self.timestep)
        self.camera.recognitionEnable(self.timestep)
        self.camera_publisher = self.create_publisher(Image, 'camera', 1)
        self.camera_info_publisher = self.create_publisher(CameraInfo, 'camera_info', 1)

        self.get_logger().info('Enabled camera')

        #depth camera
        self.depth_cam = self.robot.getDevice('range-finder')
        self.depth_cam.enable(self.timestep)
        self.depth_publisher = self.create_publisher(Image, 'depth', 1)
        
        self.get_logger().info('Enabled depth camera')

        #add accelerometer sensor here
        self.accelerometer = self.robot.getDevice('accelerometer')
        self.accelerometer.enable(self.timestep)
        self.accel_publisher = self.create_publisher(Vector3, 'accel', 1)

        self.get_logger().info('Enabled accelerometer')

        self.laser_publisher = self.create_publisher(LaserScan, '/scan', 1)
        self.tof_sensor = self.robot.getDistanceSensor('tof')
        if self.tof_sensor:
            self.tof_sensor.enable(self.timestep)
        else:
            self.get_logger().info('ToF sensor is not present for this e-puck version')

        self.motor_max_speed = self.lm.getMaxVelocity()

        #Create subscriber
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, 'cmd_vel', self.cmdVel_callback, 1
        )

        self.transform_subscriber = self.create_subscription(
            TFMessage, 'tf', self.transform_callback, 1
        )

        self.trans = TFMessage()
        self.meta_trans = TransformStamped()
        self.transform = Transform()
        self.translation = Vector3()
        self.rotation = Quaternion()


    def cmdVel_callback(self, msg):
        wheel_gap = 0.057912 # in meter
        wheel_radius = 0.02032  # in meter

        left_speed = ((2.0 * msg.linear.x - msg.angular.z *
                       wheel_gap) / (2.0 * wheel_radius))
        right_speed = ((2.0 * msg.linear.x + msg.angular.z *
                        wheel_gap) / (2.0 * wheel_radius))
        left_speed = min(self.motor_max_speed,
                         max(-self.motor_max_speed, left_speed))
        right_speed = min(self.motor_max_speed,
                          max(-self.motor_max_speed, right_speed))

        self.lm.setVelocity(left_speed)
        self.rm.setVelocity(right_speed)

    def transform_callback(self, msg):
        self.trans = msg.transforms
        self.meta_trans = self.trans[0]
        self.transform = self.meta_trans.transform

    def sensor_callback(self):
        #distance sensors
        right_distance = Float64()
        right_distance.data = self.rds.getValue()
        self.rds_publisher.publish(right_distance)

        front_distance = Float64()
        front_distance.data = self.fds.getValue()
        self.fds_publisher.publish(front_distance)

        left_distance = Float64()
        left_distance.data = self.lds.getValue()
        self.lds_publisher.publish(left_distance)

        #position sensors
        right_position = Float64()
        right_position.data = self.rps.getValue()
        self.rps_publisher.publish(right_position)

        left_position = Float64()
        left_position.data = self.lps.getValue()
        self.lps_publisher.publish(left_position)

        #imu sensor
        msg_imu = Float64()
        msg_imu.data = self.imu.getRollPitchYaw()[2]
        self.imu_publisher.publish(msg_imu)

        #camera sensor
        image = Image()
        image.data = self.camera.getImage()
        self.camera_publisher.publish(image)

        #depth camera sensor
        depth = Image()
        depth.data = self.depth_cam.getRangeImage(data_type="buffer")
        self.depth_publisher.publish(depth)

        timestamp = time.time()
        
        #try saving images here then run ORB on image dataset
        #if(self.camera.saveImage(f"rgb/{timestamp:.7f}.png", 100) == -1 or 
           #self.depth_cam.saveImage(f"depth/{timestamp:.7f}.png", 100) == -1):
            #self.get_logger().info("Issue capturing images")
            #return

        #add accelerometer sensor here
        acceleration = Vector3()
        ac = self.accelerometer.getValues()

        acceleration.x = ac[0]
        acceleration.y = ac[1]
        acceleration.z = ac[2]
        self.accel_publisher.publish(acceleration)

        self.width = self.camera.getWidth()
        self.height = self.camera.getHeight()
        self.get_logger().info("Width of camera is {}; Height of camera is {}".format(self.width, self.height))

        f = open("/home/noah/Webots/sequence/accelerometer.txt", 'a')
        #f.write("\n{:.7f} {:.7f} {:.7f} {:.7f}".format(timestamp, ac[0], ac[1], ac[2]))
        f.close()

        self.translation = self.transform.translation
        self.rotation = self.transform.rotation

        f = open("/home/noah/Webots/sequence/groundtruth.txt", 'a')
        #f.write("\n{:.7f} {:.7f} {:.7f} {:.7f} {:.7f} {:.7f} {:.7f} {:.7f}".format(
        #    timestamp,
        #    self.translation.x * 39.3701,
        #    self.translation.y * 39.3701,
        #    self.translation.z * 39.3701,
        #    self.rotation.x,
        #    self.rotation.y,
        #    self.rotation.z,
        #    self.rotation.w
        #))
        f.close()

        #laser transform
        laser_transform = TransformStamped()
        laser_transform.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        laser_transform.header.frame_id = 'base_link'
        laser_transform.child_frame_id = 'laser_scanner'
        laser_transform.transform.rotation.x = 0.0
        laser_transform.transform.rotation.y = 0.0
        laser_transform.transform.rotation.z = 0.0
        laser_transform.transform.rotation.w = 1.0
        laser_transform.transform.translation.x = 0.0
        laser_transform.transform.translation.y = 0.0
        laser_transform.transform.translation.z = 0.033


def main(args=None):
    rclpy.init(args=args)
    client_vel = ServiceNodeVelocity(args=args)
    rclpy.spin(client_vel)

    client_vel.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

