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


rospy.init_node('ini')  
pubReliabilityValue = rospy.Publisher('/reliabilityValue', Float32, queue_size=100)
pubDetectionValue = rospy.Publisher('/detectionValue', Float32, queue_size=100)
pubSpeedValue = rospy.Publisher('/speedValue', Int16, queue_size=100)

time.sleep(0.5)
pubReliabilityValue.publish(18.0)
pubDetectionValue.publish(8.3) #8.3
pubSpeedValue.publish(4)
