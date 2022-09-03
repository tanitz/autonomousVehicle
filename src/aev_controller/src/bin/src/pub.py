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
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

import rospy
from std_msgs.msg import String

pub = rospy.Publisher('/raw_datt', Float64, queue_size=10)
rospy.init_node('node_name')
r = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
   st=1.1
   pub.publish(st)
   r.sleep()