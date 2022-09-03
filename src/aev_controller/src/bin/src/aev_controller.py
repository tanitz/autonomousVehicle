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
score_reliability = 0
mode_aev = 0
station_aev = 0
steering_ctrl = 0.0
brake_req = 0
steering_ctrlTranfer = 0
steering_input  = 0
steering_setpoint = 0
last_diff_steering = 0
score_detection = 0.0
reliabilityValueCtrl = 20.0
detectionValueCtrl = 8.0
speedValueCtrl = 0
jogCtrl = 0
jogValue = 0
jogValueCtrl = 0
speed_input = 0
check_rel = 0
promlaw = 0

def callback(data):
    global steering_ctrl    
    steering_ctrl = data.cmd.steering_angle
    return steering_ctrl

def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

def reliability(data):
    global score_reliability
    score_reliability = data.data
    return score_reliability

def moveAEV(data):
    global mode_aev
    mode_aev = data.data
    return int(mode_aev)

def station(data):
    global station_aev
    station_aev = data.data
    return int(station_aev)

def s_brake(data):
    global brake_req
    brake_req = data.data
    return int(brake_req)

def pid(data):
    global steering_input, steering_setpoint,speed_input
    steering_input = data.linear.x
    steering_setpoint = data.linear.y
    speed_input = data.angular.x
    return int(steering_input),int(steering_setpoint),int(speed_input)


def detection(data):
    global score_detection
    score_detection = data.angular.z
    return score_detection

def subReliabilityValue(data):
    global reliabilityValueCtrl
    reliabilityValueCtrl =  data.data
    return reliabilityValueCtrl


def subDetectionValue(data):
    global detectionValueCtrl
    detectionValueCtrl =  data.data
    return detectionValueCtrl


def subSpeedValue(data):
    global speedValueCtrl
    speedValueCtrl =  data.data
    return speedValueCtrl

def subjog(data):
    global jogCtrl
    jogCtrl =  data.data
    return jogCtrl

def subjogValue(data):
    global jogValueCtrl
    jogValueCtrl =  data.data
    return jogValueCtrl

def submarkers_object(data):
    global markers_object_ctrl
    markers_object_ctrl =  data.data
    return markers_object_ctrl

def subtime_delay(data):
    global time_delay_ctrl
    time_delay_ctrl =  data.data
    return time_delay_ctrl

rospy.init_node('Controller_publisher')  
steering_pub = rospy.Publisher('/steering', Int16, queue_size=100)
speed_pub = rospy.Publisher('/speed', Int16, queue_size=100)
brake_pub = rospy.Publisher('/brake_AEV', Int16, queue_size=100000)
JOGon_pub = rospy.Publisher('/JOG_ON', Int16, queue_size=100000)

listener = tf.TransformListener()
rospy.loginfo('Controller connect.....')
time_start = time.time()
time_steering = time.time()
count = 0

statsBrake = 2

while not rospy.is_shutdown():
    # value = randint(-30, 30)
    rospy.Subscriber("/ctrl_cmd",ControlCommandStamped,callback) #steering_ctrl 
    rospy.Subscriber("/ndt_reliability",Float32,reliability) #score_reliability
    rospy.Subscriber("/status_go",Int16,moveAEV) #mode_aev
    rospy.Subscriber("/station_select",Int16,station) #``
    rospy.Subscriber("/status_brake",Int16,s_brake)#brake_req
    rospy.Subscriber("/angle",Twist,pid)#steering_setpoint ,steering_input
    rospy.Subscriber("/position_cmd",Twist,detection) # score_detection
    rospy.Subscriber("/reliabilityValue",Float32,subReliabilityValue) # reliabilityValueCtrl
    rospy.Subscriber("/detectionValue",Float32,subDetectionValue) # detectionValueCtrl
    rospy.Subscriber("/speedValue",Int16,subSpeedValue) # speedValueCtrl
    rospy.Subscriber("/JOG",Int16,subjog) # jogCtrl
    rospy.Subscriber("/Detect_Object",Int16,submarkers_object) # markers_object_ctrl
    rospy.Subscriber("/time_delay",Int16,subtime_delay)#time_delay_ctrl
    diff_steering = abs(steering_setpoint - steering_input)
    time.sleep(0.1)
    # try:
    #     (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #     continue\
    if (jogCtrl == 0  ):
        JOGon_pub.publish(1) 
        if (score_detection==0):
            score_detection = 20
        if(mode_aev ==1 and score_detection <= detectionValueCtrl and markers_object_ctrl == 1) or
            (mode_aev ==1 and score_detection <= 4 and markers_object_ctrl == 0) or
            (mode_aev ==1 and markers_object_ctrl == 0 and speed_input >=8 ) or
            (time_delay_ctrl = 1) or
            (brake_req == 1)
            brake_pub.publish(-1600)
            speed_pub.publish(0)
        elif(mode_aev ==0 and 
            score_detection >= detectionValueCtrl and 
            abs(score_reliability) <= reliabilityValueCtrl or 
            brake_req == 0):
            brake_pub.publish(0)
            # speed_pub.publish(0)
        if (mode_aev ==1):
            steering_ctrlTranfer = translate(steering_ctrl,-0.55,0.55,120,-120)
            steering_pub.publish(int(steering_ctrlTranfer))
        if (mode_aev ==1 and 
            score_detection >= detectionValueCtrl and 
            abs(score_reliability) <= reliabilityValueCtrl):
            speed_pub.publish(speedValueCtrl)
        elif ((score_detection <= detectionValueCtrl) or
            (time_delay_ctrl ==0) or
            (mode_aev ==0)):
            speed_pub.publish(0)
    if (jogCtrl == 1  ):
        JOGon_pub.publish(0) 

    
    print(mode_aev,reliabilityValueCtrl,detectionValueCtrl,speed_input)
