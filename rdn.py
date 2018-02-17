#!/usr/bin/env python
# -*- encoding: UTF-8 -*-
import os
import sys
import time
import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud
from math import *

class Pioneer:
    def __init__(self, rospy):
        rospy.init_node('RDN')

        self.r = 0.096
        self.R = 0.27

        self.vp_msg = Twist()
        self.cmd_vp_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber("/RosAria/pose", Odometry, self.CB_pos)
        self.sub = rospy.Subscriber("/RosAria/sonar", PointCloud, self.CB_sensors)

        self.mySin =0
        self.myCos =0
        self.myPX = 0
        self.myPY = 0
        self.myPTheta = 0

        self.activated_sensors = []
        self.mySensors = {}

    def kill(self):
        self.exit()

    def get_position(self):
        """Get the position (x,y,theta) of the robot

        Return:
            position (list): the position [x,y,theta]
        """
        position = []
        position.append(self.myPX)
        position.append(self.myPY)
        position.append(self.myPTheta)
        return position

    def CB_pos(self, msg):
        self.mySin = msg.pose.pose.orientation.z
        self.myCos = msg.pose.pose.orientation.w
        self.myPX = msg.pose.pose.position.x
        self.myPY = msg.pose.pose.position.y
        self.myPTheta = 2 * atan2(self.mySin, self.myCos)

        if (self.myPTheta > 3.1415):
            self.myPTheta = self.myPTheta - 6.2830

        #self.upd([mySin,myCos,myPX,myPY, myPTheta])

    def CB_sensors(self, msg):
        """Deal with a sensors message published on the topic.

        Args:
            (PointCloud) message to parse (http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html)
        """
        for i in range(len(msg.points)) :
            point = msg.points[i]



    # def upd(self,list):
    #     self.mySin = list[0]
    #     self.myCos = list[1]
    #     self.myPX = list[2]
    #     self.myPY = list[3]
    #     self.myPTheta = list[4]

    def set_motor_velocity(self, control):
        """Set a target velocity on the pioneer motors, multiplied by the gain
        defined in self.gain

        Args:
            control(list): the control [left_motor, right_motor]
        """
        vg = control[0]*self.r
        vd = control[1]*self.r
        self.vp_msg.linear.x = 10*0.5*(vg+vd)
        self.vp_msg.angular.z= 0.5*(vd-vg)/self.R
        #print ("pubishing velocity" , str(self.vp_msg.linear.x) ," , ",self.vp_msg.angular.z)
        self.cmd_vp_pub.publish(self.vp_msg)

    def get_proximity_sensors(self):
        """Get values for all proximity sensors.

        """
        values = [0 for i in range(len(self.proximity_sensors))];
        for sensor in range(len(self.proximity_sensors)):
            values[sensor] = self.get_proximity_sensor(sensor)
        return values

    def get_proximity_sensor(self, sensor):
        """Get value of given proximity sensor.

        Args:
            sensor(int): the index of the sensor to get value of.
        """
        return vrep.simxReadProximitySensor(self.client_id, self.proximity_sensors[sensor], vrep.simx_opmode_buffer);

    def get_sensors_distances(self):
        """ Returns the distances of all the sensors in a list.

        Args:
            (list): sensors to get distance of.

        Return:
            (list): the sensors distances.
        """
        sensors_values = self.get_proximity_sensors()

        invert_distances = []
        for value in sensors_values :
            vector = value[2]
            distance = math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)
            if (value[1]) :
                if (distance == 0) :
                    invert_distances.append(100)
                else :
                    invert_distances.append(1 / distance)
            else :
                invert_distances.append(0)

        return invert_distances
