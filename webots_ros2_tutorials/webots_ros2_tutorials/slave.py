# Copyright 1996-2020 Soft_illusion.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from webots_ros2_core.webots_node import WebotsNode
import math
from std_msgs.msg import Float64
from rclpy.time import Time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster,TransformBroadcaster 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class ServiceNodeVelocity(WebotsNode):
    def __init__(self, args):
        super().__init__('slave_node', args)

        # Enable 3 sensors
        self.service_node_vel_timestep = 32

        # Sensor section
        self.sensor_timer = self.create_timer(
            0.001 * self.service_node_vel_timestep, self.sensor_callback)

        self.right_sensor = self.robot.getDistanceSensor(
            'distance_sensor_right')
        self.right_sensor.enable(self.service_node_vel_timestep)
        self.sensor_publisher_right = self.create_publisher(Float64, 'right_IR', 1)

        self.mid_sensor = self.robot.getDistanceSensor('distance_sensor_mid')
        self.mid_sensor.enable(self.service_node_vel_timestep)
        self.sensor_publisher_mid = self.create_publisher(Float64, 'mid_IR', 1)

        self.left_sensor = self.robot.getDistanceSensor('distance_sensor_left')
        self.left_sensor.enable(self.service_node_vel_timestep)
        self.sensor_publisher_left = self.create_publisher(
            Float64, 'left_IR', 1)

        

        # Front wheels
        self.left_motor_front = self.robot.getMotor('left_front_wheel')
        self.left_motor_front.setPosition(float('inf'))
        self.left_motor_front.setVelocity(0)

        self.right_motor_front = self.robot.getMotor('right_front_wheel')
        self.right_motor_front.setPosition(float('inf'))
        self.right_motor_front.setVelocity(0)

        # Rear wheels
        self.left_motor_rear = self.robot.getMotor('left_rear_wheel')
        self.left_motor_rear.setPosition(float('inf'))
        self.left_motor_rear.setVelocity(0)

        self.right_motor_rear = self.robot.getMotor('right_rear_wheel')
        self.right_motor_rear.setPosition(float('inf'))
        self.right_motor_rear.setVelocity(0)

        self.motor_max_speed = self.left_motor_rear.getMaxVelocity()

        # Create Subscriber
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, 'cmd_vel', self.cmdVel_callback, 1)

        # Create Lidar subscriber
        self.lidar_sensor = self.robot.getLidar('lidar_sensor')
        self.lidar_sensor.enable(self.service_node_vel_timestep)
        self.laser_publisher = self.create_publisher(LaserScan, '/scan', 1)

        # self.publish_tf()

        # self.publish_odom()
        ##########################
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.time_step=0.032
        self.odom_pub = self.create_publisher(Odometry,"odom",1)
        self.odom_timer = self.create_timer(self.time_step, self.odom_callback)
        #########################
        self.get_logger().info('Sensor enabled')

    ####################################
    def odom_callback(self):
        self.publish_odom()


    def euler_to_quaternion(self, yaw, pitch, roll):

        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        return [qx, qy, qz, qw]

    def publish_odom(self):
        self.odom_broadcaster = TransformBroadcaster(self)
        # self.current_time = datetime.now().microsecond
        # compute odometry in a typical way given the velocities of the robot
        dt = self.time_step
        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        # odom_quat=[0.0,0.0,0.0,1.0]
        odom_quat=self.euler_to_quaternion(self.th,0,0)

        # first, we'll publish the transform over tf
        odom_transform = TransformStamped()
        odom_transform.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        odom_transform.header.frame_id = 'base_link'
        odom_transform.child_frame_id = 'odom'
        odom_transform.transform.rotation.x = odom_quat[0]
        odom_transform.transform.rotation.y = odom_quat[1]
        odom_transform.transform.rotation.z = odom_quat[2]
        odom_transform.transform.rotation.w = odom_quat[3]
        odom_transform.transform.translation.x = self.x
        odom_transform.transform.translation.y = self.y
        odom_transform.transform.translation.z = 0.0

        self.odom_broadcaster.sendTransform(odom_transform)

        odom = Odometry()
        odom.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        # set the position
        odom.pose.pose.position.x= self.x
        odom.pose.pose.position.y= self.y
        odom.pose.pose.orientation.x=odom_quat[0]
        odom.pose.pose.orientation.y=odom_quat[1]
        odom.pose.pose.orientation.z=odom_quat[2]
        odom.pose.pose.orientation.w= odom_quat[3]

        # set the velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z=self.vth

        # publish the message
        self.odom_pub.publish(odom)
    ###################################
    # def publish_tf(self):
    #     # tf publish for map
    #     self.static_broadcaster = StaticTransformBroadcaster(self)
    #     map_transform = TransformStamped()
    #     map_transform.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
    #     map_transform.header.frame_id = 'map'
    #     map_transform.child_frame_id = 'base_link'
    #     map_transform.transform.rotation.x = 0.0
    #     map_transform.transform.rotation.y = 0.0
    #     map_transform.transform.rotation.z = 0.0
    #     map_transform.transform.rotation.w = 1.0
    #     map_transform.transform.translation.x = 0.2
    #     map_transform.transform.translation.y = 0.0
    #     map_transform.transform.translation.z = 0.0
        # self.static_broadcaster.sendTransform(map_transform)

    def cmdVel_callback(self, msg):
        wheel_gap = 0.1  # in meter
        wheel_radius = 0.04  # in meter
        self.vx = msg.linear.x
        self.vth = msg.angular.z
        left_speed = ((2.0 * msg.linear.x - msg.angular.z *
                       wheel_gap) / (2.0 * wheel_radius))
        right_speed = ((2.0 * msg.linear.x + msg.angular.z *
                        wheel_gap) / (2.0 * wheel_radius))
        left_speed = min(self.motor_max_speed,
                         max(-self.motor_max_speed, left_speed))
        right_speed = min(self.motor_max_speed,
                          max(-self.motor_max_speed, right_speed))

        self.left_motor_front.setVelocity(left_speed)
        self.right_motor_front.setVelocity(right_speed)
        self.left_motor_rear.setVelocity(left_speed)
        self.right_motor_rear.setVelocity(right_speed)

    def sensor_callback(self):
        # Publish distance sensor value
        msg_right = Float64()
        msg_right.data = self.right_sensor.getValue()
        self.sensor_publisher_right.publish(msg_right)

        msg_mid = Float64()
        msg_mid.data = self.mid_sensor.getValue()
        self.sensor_publisher_mid.publish(msg_mid)

        msg_left = Float64()
        msg_left.data = self.left_sensor.getValue()
        self.sensor_publisher_left.publish(msg_left)

        self.laser_pub()



    def laser_pub(self):
        msg_lidar = LaserScan()
        msg_lidar.header.frame_id = 'base_link'
        stamp = Time(seconds=self.robot.getTime()).to_msg()
        msg_lidar.header.stamp = stamp
        msg_lidar.angle_min = 0.0
        msg_lidar.angle_max = 2 * 22 / 7
        msg_lidar.angle_increment = ( 2 * 22 ) / (180 * 7 )
        msg_lidar.range_min = 0.12
        msg_lidar.range_max = 0.5
        msg_lidar.scan_time = 0.032
        msg_lidar.ranges = self.lidar_sensor.getRangeImage()

        self.laser_publisher.publish(msg_lidar)




def main(args=None):
    rclpy.init(args=args)
    client_vel = ServiceNodeVelocity(args=args)

    rclpy.spin(client_vel)

    client_vel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
