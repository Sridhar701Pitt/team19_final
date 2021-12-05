#!/usr/bin/env python

from ROS.catkin_ws.src.team19_final.src.enums import Quadrant
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import enums

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class move_robot:
    '''
    move robot forward
    rotate robot by specified angle
    rotates robot precisely to stay parallel/perpendicular to the specified wall
    find if the robot is facing the wall

    inputs: lidar scan,
    '''
    def __init__(self):
        rospy.init_node('move_robot_node',anonymous=True)
        self.move_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size = 10)
        self.lidar_scan = rospy.Subscriber('/scan', LaserScan, self.laser_subscriber, queue_size = 10)
        self.max_scan_distance = 0.7
        self.scan_array = np.empty((1, 360))
        self.fov = 30
        self.desired_distance = 0.1
        self.desired_distance_error = 0.01
        self.odom_linear = 0.0
        self.odom_angular = 0.0
        self.odom_orientation = Quaternion()
        self.max_velocity = 0.22
        self.angular_adjustment_error = 0.05

    def rotate_robot(self, direction)
        # TODO

        while True:

            command_vel = Twist()

            orientation_list = [self.odom_orientation.x, self.odom_orientation.y, self.odom_orientation.z, self.odom_orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

            p_param = 1

            if direction == 1:
                yaw_target = yaw - np.pi / 2

                command_vel.angular.z = p_param * 




    def rotate_robot_precise(self, quadrant):
        '''
        Rotates robot precisely upon to line up with walls
        '''
        while True:
            command_vel = Twist()

            _, quad_array = self.quadrant_array(quadrant)
            [array_left, array_right] = np.array_split(quad_array,2)

            if -1 in quad_array:
                raise Exception("invalid element '-1' in quad array")

            if (np.average(array_right) - np.average(array_left)) < self.angular_adjustment_error:
                command_vel.angular.z = 0
                self.move_publisher.publish(command_vel)
                break
            
            # angular adjustment controller
            p_param = 1
            command_vel.angular.z = p_param*((np.average(array_right) - np.average(array_left)) - self.odom_angular) + self.odom_angular

            self.move_publisher.publish(command_vel)

    def move_robot_forward(self):
        '''
        Moves robot forward until desired distance is reached

        '''
        
        
        while True:
            command_vel = Twist()

            _, distance_n = self.is_facing_wall(Quadrant.N)
            _, distance_e = self.is_facing_wall(Quadrant.E)
            _, distance_w = self.is_facing_wall(Quadrant.W)

            # linear controller
            if distance_n == -1:
                distance_n = self.max_scan_distance

            p1_param = 1.5
            p2_param = 0.7
            command_vel.linear.x = np.min(p1_param*(p2_param*(distance_n - self.desired_distance) - self.odom_linear) + self.odom_linear, self.max_velocity)

            # angular adjustment controller
            if distance_w != -1:
                p_param = 1
                command_vel.angluar.z = p_param*((distance_w - self.desired_distance) - self.odom_angular) + self.odom_angular
            elif distance_e != -1:
                p_param = 1
                command_vel.angluar.z = p_param*((self.desired_distance - distance_e) - self.odom_angular) + self.odom_angular
            else:
                command_vel.angular.z = 0

            # stop if in range
            if self.desired_distance - self.desired_distance_error <= distance_n <= self.desired_distance + self.desired_distance_error:
                command_vel.linear.x = 0
                command_vel.angular.z = 0
                self.move_publisher.publish(command_vel)
                break
            
            self.move_publisher.publish(command_vel)

    def odom_callback(self, odom_object):
        self.odom_linear = odom_object.twist.twist.linear.x
        self.odom_angular = odom_object.twist.twist.angular.z
        self.odom_orientation = odom_object.pose.pose.orientation

    def is_facing_wall(self,quadrant):
        '''
        Function to find if the specified quadrant contaisn a wall and the distance to that wall 
        Args:
            quadrant: enum variable which holds the direction flag values
        
        Return:
            is_facing_wall: Bool value which denotes if the robot is facing a wall or not
            dist_to_wall: float value which gives the distance to the wall that the robot is facing
        '''
        if not isinstance(quadrant, Quadrant):
            raise TypeError('quadrant must be an instance of Quadrant Enum')
        
        _, quad_array = self.quadrant_array(quadrant)
        if -1 in quad_array:
            return False, -1
        else:
            scan_distance = np.average(quad_array)
            if self.desired_distance - self.desired_distance_error <= scan_distance <= self.desired_distance + self.desired_distance_error:
                return True, scan_distance
            else:
                return False, scan_distance

    def laser_subscriber(self, laser_scan_object):
        range_min = laser_scan_object.range_min
        range_max = min(self.max_scan_distance, laser_scan_object.range_max)
        # convert to numpy array
        self.scan_array = np.asarray(laser_scan_object.ranges)
        # make laser scan points outside range = -1
        self.scan_array = np.where(self.scan_array > range_max 
                                or self.scan_array < range_min, -1, self.scan_array)

    # Outputs the range values in each quadrant of specified fov
    def quadrant_array(self, quadrant):
        if not isinstance(quadrant, Quadrant):
            raise TypeError('quadrant must be an instance of Quadrant Enum')

        array_length = np.size(self.scan_array)

        indices = range(int(quadrant*(array_length/4) - self.fov/2), 
                        int(quadrant*(array_length/4) + self.fov/2))

        return indices, self.scan_array.take(indices, mode='wrap')