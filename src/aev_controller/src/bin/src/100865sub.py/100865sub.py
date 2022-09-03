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
from autoware_msgs.msg import ControlCommand, SteerCmd, ControlCommandStamped ,BrakeCmd,NDTStat
from std_msgs.msg import Int16 , Float32
from visualization_msgs.msg import Marker, MarkerArray

# global marker_ests
marker_ests = []
a1 = ""
brake_req = 0


def markers(data):
    global marker_ests ,a1
    i = 0
    marker_ests = []
    for i in range(len(data.markers)-1):
        a1 = data.markers[i].pose.position.x
        # print(a1)
        marker_ests.append(a1)
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


score_autoware = 0
time_autoware = 0
score_reliability = 0.0
time_ndtmatching = 0.0
mode_aev = 0

rospy.init_node('node_name')
listener = tf.TransformListener()
msg = Twist()
# goal_sub = rospy.Subscriber("/detection/lidar_detector/objects_markers", MarkerArray, callback)
rospy.Subscriber("/detection/lidar_detector/objects_markers", MarkerArray, markers)
brake_pub = rospy.Publisher('/brake_AEV', Int16, queue_size=100000)

msg.linear.x = 
pub_pos = rospy.Publisher('/position_cmd', Twist, queue_size=100)
while not rospy.is_shutdown():
    rospy.Subscriber("/status_brake",Int16,s_brake)#brake_req 
    rospy.Subscriber("/status_go",Int16,moveAEV) #mode_aevccccc
    # rospy.Subscriber("/brake_cmd",BrakeCmd,callback)
    # rospy.Subscriber("/ndt_stat",socreautoware,callback)
    try:
        (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        msg.linear.x = trans[0]
        msg.linear.y = trans[1]
        msg.linear.x = trans[2]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.x = 0
    
    try :
       
        dObject = min(b for b in marker_ests if b > 0)
    
        msg.angular.z = dObject
       
        # print min(marker_ests)
    except:
        dObject = 0.0
        msg.angular.z = dObject

    try:
        pub_pos.publish(msg)
        print (dObject,trans)
    except:
        print("not detect")
    sleep(0.1)

    # if(brake_req == 1 ):
            
    #         brake_pub.publish(-24000)
    # if(brake_req == 0 and mode_aev == 0 ) :
           
    #         brake_pub.publish(0)



    
    # if(score_reliability <= 15):
        
    # else:

    # move = translate(brake_ctrl,0,10000,0,-24000)
    # pub.publish(int(move))
    # print(str(score_reliability)+"  "+str(time_ndtmatching)+"  "+str(trans))
    # print(marker_ests)
    