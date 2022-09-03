#!/usr/bin/env python

import math
from math import sin, cos, pi ,tan
import time
from time import sleep
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from pyModbusTCP.client import ModbusClient
import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from autoware_msgs.msg import ControlCommand, SteerCmd, ControlCommandStamped ,BrakeCmd,NDTStat
from std_msgs.msg import Int16 , Float32
sensortf = 0
tf_score = 0
# def sensorTF(data):
#     global tf_score
#     tf_score = data.vector.x
#     return int(tf_score)

def sensorTF(data):
    global tf_score
    tf_score = data.linear.x
    return int(tf_score)

while not rospy.is_shutdown():
    rospy.Subscriber("/sensorArduino",Twist,sensorTF)#tf_score


    if (tf_score>20):
        sensortf = 1
        print(sensortf,tf_score)

    elif (tf_score<10):
        sensortf = 0
        print(sensortf,tf_score)