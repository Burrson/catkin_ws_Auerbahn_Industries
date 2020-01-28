#!/usr/bin/env python

import rospy
import time
import math
import numpy as np

from autominy_msgs.msg import NormalizedSpeedCommand, NormalizedSteeringCommand, SteeringFeedback, Speed
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

speed = 0.2
speed_cmd = NormalizedSpeedCommand()
steering_cmd = NormalizedSteeringCommand()
speed_cmd.value = speed
punkt = Odometry()
close = Marker()
clicked = Marker()
lookahead = Marker()
isactiv = False
steeringvalue = 0.0
gewollterWinkel = 0.0
lastgewollterWinkel = 0.0
kurvenverlangsamung = 1.0 #Muss noch angepasst werden, konnte aber empierisch nicht getan werden, da das gps aus war
kp = 4.0
ki = 0.1
kd = 0.2
lasterror = 0.0
ierror = 0.0
derror = 0.0
dt = 1.0

def fangePunkt(x):
    global punkt
    punkt = x


def fangeclose(x):
    global close
    close = x

def fangeclicked(x):
    global clicked
    clicked = x
    steuerung()

def fangeLookahead(x):
    global lookahead
    lookahead = x
    #print (x.pose.position.x)
    #print (x.pose.position.y)

def steuerung():
    global gewollterWinkel, lastgewollterWinkel, isactiv, punkt, lookahead, clicked, speed
    if(not isactiv):
        isactiv = True
        lastgewollterWinkel = gewollterWinkel
        gewollterWinkel = 1*(math.atan2(clicked.pose.position.y-lookahead.pose.position.y,clicked.pose.position.x-lookahead.pose.position.x))
        if (gewollterWinkel >= 0):
            gewollterWinkel = gewollterWinkel - 3.141
        else:
            gewollterWinkel = gewollterWinkel + 3.141

        korrigiereWinkel(math.atan2(2*punkt.pose.pose.orientation.w*punkt.pose.pose.orientation.z,1-2*(punkt.pose.pose.orientation.z*punkt.pose.pose.orientation.z)))
        ziel = Odometry()
        #ziel = punkt
        ziel.header.frame_id = "map"
        #ziel.ns = "richtung"
        #ziel.id = 0;
        ziel.pose.pose.position.x = punkt.pose.pose.position.x
        ziel.pose.pose.position.y = punkt.pose.pose.position.y
        ziel.pose.pose.position.z = punkt.pose.pose.position.z
        ziel.pose.pose.orientation.x = 0.0
        ziel.pose.pose.orientation.y = 0.0
        ziel.pose.pose.orientation.z = math.sin(gewollterWinkel/2.0)
        ziel.pose.pose.orientation.w = math.cos(gewollterWinkel/2.0)

        pub_Odo.publish(ziel)
        print (gewollterWinkel, lastgewollterWinkel)
        #print(punkt.pose.pose.orientation.z*3.1)

        isactiv = False


def korrigiereWinkel(winkel):
    global gewollterWinkel, steeringvalue, lasterror, derror, ierror, dt
    if abs(gewollterWinkel-winkel) > 0.00:
        error = gewollterWinkel-winkel
        error = math.atan2(math.sin(error), math.cos(error))

        if error > 3.141:
            error = (error-6.283)
        if error < -3.141:
            error = (error+6.283)

        ierror += error * ki / dt

        if ierror > 1.0:
            ierror = 1.0
        if ierror < -1.0:
            ierror = -1.0

        derror = (error - lasterror) * kd / dt #* 10.0 * speed

        steeringvalue = error*kp + ierror + derror
        #print ("error:",error)
        if steeringvalue > 1.0:
            steeringvalue = 1.0
        if steeringvalue < -1.0:
            steeringvalue = -1.0

        steering_cmd.value = steeringvalue

        lasterror = error
    else:
        steering_cmd.value = 0.0
    #print("winkel",steering_cmd.value)
    pub_steering.publish(steering_cmd)


def korrigiereWinkel2(winkel):
    global gewollterWinkel, steeringvalue, lasterror
    if abs(gewollterWinkel-winkel) > 0.00:
        error = gewollterWinkel-winkel

        if error > 3.141:
            error = (error-6.283)
        if error < -3.141:
            error = (error+6.283)

        steeringvalue += ((4*error)-steeringvalue)*0.5

        print ("error:",error)

        if steeringvalue > 1.0:
            steeringvalue = 1.0
        if steeringvalue < -1.0:
            steeringvalue = -1.0

        steering_cmd.value = steeringvalue

        lasterror = error
    else:
        steering_cmd.value = 0.0
    #print("winkel",steering_cmd.value)
    pub_steering.publish(steering_cmd)

def speedcontroller(realspeed):
    global speed
    speed_cmd.value = speed/curvature()
    #print (realspeed.value,speed_cmd.value,speed)
    pub_speed.publish(speed_cmd)

def curvature():
    kruemmung = 1.0
    kruemmung += abs(lastgewollterWinkel-gewollterWinkel)*kurvenverlangsamung
    return kruemmung



rospy.init_node("control")



pub_steering = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
pub_speed = rospy.Publisher("/actuators/speed_normalized", NormalizedSpeedCommand, queue_size=100)
sub_point = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, fangePunkt, queue_size=10)
sub_speed = rospy.Subscriber("/sensors/speed", Speed, speedcontroller, queue_size=10)
sub_lookahead = rospy.Subscriber("/spline/lookahead", Marker, fangeLookahead, queue_size=10)
sub_close = rospy.Subscriber("/spline/close", Marker, fangeclose, queue_size=10)
sub_clicked = rospy.Subscriber("/spline/clicked", Marker, fangeclicked, queue_size=10)
pub_Odo = rospy.Publisher("/control/Odo", Odometry, queue_size=10)

def main():
    print ("main")

if __name__ == '__main__':
    main()

rospy.spin()
