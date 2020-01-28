#!/usr/bin/env python

import rospy
import time
import math
import numpy as np

from autominy_msgs.msg import NormalizedSpeedCommand, NormalizedSteeringCommand, SteeringFeedback
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

speed = 0.1
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
kp = 0.5
kd = 1.0
lasterror = 0.0

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
    global gewollterWinkel, isactiv, punkt, lookahead, clicked
    if(not isactiv):
        isactiv = True
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
        #print (gewollterWinkel)
        #print(punkt.pose.pose.orientation.z*3.1)

        pub_speed.publish(speed_cmd)
        isactiv = False


def korrigiereWinkel(winkel):
    global gewollterWinkel, steeringvalue, lasterror
    #print (gewollterWinkel)
    #print (winkel)
    #if (gewollterWinkel > 6.2):
    #    gewollterWinkel = 0.0
    if abs(gewollterWinkel-winkel) > 0.00:
        error = gewollterWinkel-winkel

        if error > 3.141:
            error = (error-6.283)
        if error < -3.141:
            error = (error+6.283)

        steeringvalue += ((4*error)-steeringvalue)*0.5

        #if error > 0:
        #    steeringvalue += kp
        #if error < 0:
        #    steeringvalue -= kp

        print ("error:",error)
        #if steeringvalue > abs(error*1.3):
            #steeringvalue = abs(error*1.3)
        #if steeringvalue < -abs(error*1.3):
            #steeringvalue = -abs(error*1.3)

        if steeringvalue > 1.0:
            steeringvalue = 1.0
        if steeringvalue < -1.0:
            steeringvalue = -1.0


        #steeringvalue = steeringvalue - (kd*((lasterror-error)))


        steering_cmd.value = steeringvalue

        lasterror = error
    else:
        steering_cmd.value = 0.0
    print("winkel",steering_cmd.value)
    pub_steering.publish(steering_cmd)


def korrigiereWinkel2(winkel):
    global gewollterWinkel, steeringvalue, lasterror

    if (gewollterWinkel > 6.283):
        gewollterWinkel = 0.0
    if abs(gewollterWinkel-winkel) > 0.0:
        error = gewollterWinkel-winkel
        if error > 3.141:
            error = (error-6.283)
        print("error:", error)
        #Das P im PD controller
        steeringvalue = steeringvalue + (kp*((error/3.141)*-1))
        print ("lenkaenderung",(kp*((error/3.141)*-1)))
        if steeringvalue > 1.0:
            steeringvalue = 1.0
        if steeringvalue < -1.0:
            steeringvalue = -1.0

        #Das D im PD controller
        steeringvalue = steeringvalue + (kd*((lasterror-error)/3.141))
        print("Daemfung:",(kd*((lasterror-error)/3.141)))



        steering_cmd.value = steeringvalue

        lasterror = error
#        if gewollterWinkel-winkel < 0:
#            steering_cmd.value = -1.0
#            print ("zuweit rechts")
#        if gewollterWinkel-winkel > 0:
#            steering_cmd.value = 1.0
#            print ("zuweit links")
    else:
        steering_cmd.value = 0.0
    pub_steering.publish(steering_cmd)






rospy.init_node("control")



pub_steering = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
pub_speed = rospy.Publisher("/actuators/speed_normalized", NormalizedSpeedCommand, queue_size=100)
sub_point = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, fangePunkt, queue_size=10)
sub_lookahead = rospy.Subscriber("/spline/lookahead", Marker, fangeLookahead, queue_size=10)
sub_close = rospy.Subscriber("/spline/close", Marker, fangeclose, queue_size=10)
sub_clicked = rospy.Subscriber("/spline/clicked", Marker, fangeclicked, queue_size=10)
pub_Odo = rospy.Publisher("/control/Odo", Odometry, queue_size=10)

def main():
    print ("main")

if __name__ == '__main__':
    main()

rospy.spin()
