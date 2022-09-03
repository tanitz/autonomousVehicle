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

rospy.init_node('ini')  
Pubpromlaw = rospy.Publisher('/time_delay', Float32, queue_size=100)

score_reliability = 0
reliabilityValueCtrl = 20.0
check_rel = 0
promlaw = 0


def reliability(data):
    global score_reliability
    score_reliability = data.data
    return score_reliability

def subReliabilityValue(data):
    global reliabilityValueCtrl
    reliabilityValueCtrl =  data.data
    return reliabilityValueCtrl

while not rospy.is_shutdown():
    rospy.Subscriber("/ndt_reliability",Float32,reliability) #score_reliability
    rospy.Subscriber("/reliabilityValue",Float32,subReliabilityValue) # reliabilityValueCtrl
    time.sleep(0.1)

if (abs(score_reliability) >= reliabilityValueCtrl):
    while check_rel < 2:
        check_rel = check_rel + 1
        time.sleep (0.5)
    Pubpromlaw.publish(1)
elif (abs(score_reliability) <= reliabilityValueCtrl):
    check_rel = 0
    Pubpromlaw.publish(0)
        