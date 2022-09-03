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


score_autoware = 0
time_autoware = 0
score_reliability = 0.0
time_ndtmatching = 0.0
def callback(data):
    global brake_ctrl   
    brake_ctrl = data.brake
    return brake_ctrl


def reliability(data):
    global score_reliability  
    score_reliability = data.data
    return score_reliability

def time_ndt(data):
    global time_ndtmatching  
    time_ndtmatching = data.data
    return time_ndtmatching


pub = rospy.Publisher('/brake_AEV', Int16, queue_size=50000)
rospy.init_node('node_name')
listener = tf.TransformListener()
while not rospy.is_shutdown():
     
    # rospy.Subscriber("/brake_cmd",BrakeCmd,callback)
    # rospy.Subscriber("/ndt_stat",socreautoware,callback)
    rospy.Subscriber("/ndt_reliability",Float32,reliability)
    rospy.Subscriber("/time_ndt_matching",Float32,time_ndt)
    try:
        (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    # if(score_reliability <= 15):
        
    # else:

    # move = translate(brake_ctrl,0,10000,0,-24000)
    # pub.publish(int(move))
    print(str(score_reliability)+"  "+str(time_ndtmatching)+"  "+str(trans))
    