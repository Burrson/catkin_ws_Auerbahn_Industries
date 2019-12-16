#!/usr/bin/env python

import rospy
import time
import math

from autominy_msgs.msg import NormalizedSpeedCommand, NormalizedSteeringCommand, SteeringFeedback
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose

speed = 0.1
speed_cmd = NormalizedSpeedCommand()
steering_cmd = NormalizedSteeringCommand()
speed_cmd.value = speed

steeringvalue = 0.0
gewollterWinkel = 3.1
kp = 0.1
kd = 3.0

lasterror = 0.0

def fangeGPS(odo):
    curX = odo.pose.pose.position.x
    curY = odo.pose.pose.position.y
    curZ = odo.pose.pose.position.z

    oriX = odo.pose.pose.orientation.x
    oriY = odo.pose.pose.orientation.y
    oriZ = odo.pose.pose.orientation.z
    oriW = odo.pose.pose.orientation.w
    print ("Winkel im Bogenmass:", wandleQuadInWinkel(oriX,oriY,oriZ,oriW))
    korrigiereWinkel(wandleQuadInWinkel(oriX,oriY,oriZ,oriW))


def wandleQuadInWinkel(x,y,z,w):
    winkel = math.acos(z)*2
    return winkel

def korrigiereWinkel(winkel):
    global gewollterWinkel, steeringvalue, lasterror

    if (gewollterWinkel > 6.2):
        gewollterWinkel = 0.0
    if abs(gewollterWinkel-winkel) > 0.0:
        error = gewollterWinkel-winkel
        if error > 3.1:
            error = (error-6.2)
        print("error:", error)
        #Das P im PD controller
        steeringvalue = steeringvalue + (kp*((error/3.1)*-1))
        print ("lenkaenderung",(kp*((error/3.1)*-1)))
        if steeringvalue > 1.0:
            steeringvalue = 1.0
        if steeringvalue < -1.0:
            steeringvalue = -1.0

        #Das D im PD controller
        steeringvalue = steeringvalue + (kd*((lasterror-error)/3.1))
        print("Daemfung:",(kd*((lasterror-error)/3.1)))



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

rospy.init_node("gedaempftes_control")

pub_steering = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=100)
pub_speed = rospy.Publisher("/actuators/speed_normalized", NormalizedSpeedCommand, queue_size=100)
sub_GPS = rospy.Subscriber("/communication/gps/22", Odometry, fangeGPS, queue_size=10)


def main():
    global speed_cmd, steering_cmd, speed

    #time.sleep(2)
    #steering_cmd.value = -1.0
    #pub_steering.publish(steering_cmd)
    time.sleep(2)
    speed_cmd.value = 0.1
    pub_speed.publish(speed_cmd)



if __name__ == '__main__':
    main()

rospy.spin()
