#!/usr/bin/env python

import rospy
import time
import math
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand, SteeringFeedback
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose
from std_msgs.msg import Float32

stuffdone = True
curX = 0.0
curY = 0.0
curZ = 0.0
isactive = False






def fangeGPS(odo):
    global curX, curY, curZ, stuffdone, isactive

    curX = odo.pose.pose.position.x
    curY = odo.pose.pose.position.y
    curZ = odo.pose.pose.position.z


def fangeWinkel(winkel):
    pub_steeringAngle.publish((float(abs(335-winkel.value))/250))
    return

def berechneRadius((x,y),(x2,y2),(x3,y3)):
    if x2-x==0.0:
        x=x+0.01
    if x3-x==0.0:
        x=x+0.01
    if y2-y==0.0:
        y=y+0.01
    if y3-y==0.0:
        y=y+0.01
    steigungA = -(1/((y2-y)/(x2-x)))
    steigungB = -(1/((y3-y)/(x3-x)))

    if steigungA-steigungB == 0.0:
        steigungB = steigungB + 0.01

    newX = (-(steigungA*x2)+y2+(steigungB*x3)-y3)/(steigungB-steigungA)
    newY = (steigungA*newX)+(steigungA*(-x2))+y2
 
    return (math.sqrt((x-newX)**2+(y-newY)**2))/2

def berechneEinlenkWinkel(radius):
    #distanz von vorderrad zu hinterrad = 27cm
    return math.degrees(math.atan(0.27/radius))

rospy.init_node("kreise")

sub_angle = rospy.Subscriber("/sensors/arduino/steering_angle", SteeringFeedback, fangeWinkel, queue_size=10)
sub_GPS = rospy.Subscriber("/communication/gps/7", Odometry, fangeGPS, queue_size=10)
pub_steering = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=100)
pub_speed = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=100)
pub_steeringAngle = rospy.Publisher("/kreise", Float32, queue_size=100)

def main():
    while curX == 0.0:
        pass
    pos1 = (curX,curY)
    time.sleep(1)
    steering_cmd = NormalizedSteeringCommand()
    steering_cmd.value = 1.0
    pub_steering.publish(steering_cmd)
    time.sleep(1)
    speed_cmd = SpeedCommand()
    speed_cmd.value = 0.2
    pub_speed.publish(speed_cmd)
    time.sleep(4)
    speed_cmd.value = 0.0
    pub_speed.publish(speed_cmd)
    time.sleep(2)
    pos2 = (curX,curY)
    speed_cmd.value = 0.2
    pub_speed.publish(speed_cmd)
    time.sleep(4)
    speed_cmd.value = 0.0
    pub_speed.publish(speed_cmd)
    time.sleep(2)
    pos3 = (curX,curY)
    print "Radius:",berechneRadius(pos1,pos2,pos3),"Meter"
    print "Winkel:",berechneEinlenkWinkel(berechneRadius(pos1,pos2,pos3)),"Grad, weil niemand was mit Bogenmass anfangen kann"

if __name__ == '__main__':
    main()



rospy.spin()
