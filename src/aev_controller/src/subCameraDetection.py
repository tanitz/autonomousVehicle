#!/usr/bin/env python
import math
from math import sin, cos, pi ,tan
import time
from time import sleep
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3 ,PoseArray
from pyModbusTCP.client import ModbusClient
import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from autoware_msgs.msg import ControlCommand, SteerCmd, ControlCommandStamped ,BrakeCmd,NDTStat,DetectedObjectArray
from std_msgs.msg import Int16 , Float32
from visualization_msgs.msg import Marker, MarkerArray

# global marker_ests
marker_ests = []
a1 = ''
brake_req = 0


def markers(data):
    global marker_ests , a1
    i = 0
    marker_ests = []
    for i in range(len(data.objects)):
        a1 = data.objects[i].label
        
        # print(a1)
        # marker_ests += a1
        marker_ests.append(a1)
    # print(len(data.objects))
    # print(marker_ests)
    return (marker_ests)

def s_brake(data):
    global brake_req
    brake_req = data.data
    return int(brake_req)

def moveAEV(data):
    global mode_aev
    mode_aev = data.data
    return int(mode_aev)

# def markers(data):
#     global markers_object
#     markers_object = data.data
#     return int(markers_object)

pubdetect_object = rospy.Publisher('/Detect_Object', Int16, queue_size=100)

score_autoware = 0
time_autoware = 0
score_reliability = 0.0
time_ndtmatching = 0.0
mode_aev = 0

rospy.init_node('node_name1')

while not rospy.is_shutdown():
    rospy.Subscriber("/detection/image_detector/objects", DetectedObjectArray, markers)

    sleep(0.01)
    if(len(marker_ests) == 0):
# a1.publish(0)
        print("Not found")
        pubdetect_object.publish(0)
    else:
        print("car or person")
        pubdetect_object.publish(1)
        
 
        