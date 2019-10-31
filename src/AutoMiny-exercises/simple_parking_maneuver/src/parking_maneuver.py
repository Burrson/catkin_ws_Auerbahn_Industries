#!/usr/bin/env python

# --- imports ---
import rospy
from simple_parking_maneuver.srv import *
from std_msgs.msg import Float32
from std_msgs.msg import String

# --- definitions ---
last_driving_control_info = ""


def callbackDrivingControl(msg):
    last_driving_control_info = msg.data

def dasBesteEinparkenEverz():

    # Ziemlich selbsterklärend, tatsächlich einfach "right" mit anderen Werten
    driving_direction_pub1 = pub_back_right
    driving_direction_pub2 = pub_back_left
    driving_direction_pub3 = pub_forward

    
    driving_direction_pub1.publish(0.25)
    rospy.sleep(5)
    driving_direction_pub3.publish(0.40)
    rospy.sleep(5)    
    driving_direction_pub4.publish(0.8)
    
    return

def dasAlphaUndOmega():

    # Sie hat einmal funktioniert. Diese verdammte Funktion hat einmal funktioniert
    # und da lief die verdammte Kamera nicht. 
    # Sie sollte das Auto um zwei Schachfiguren im Slalom umfahren und dann
    # geradezu in eine Packlücke...
    # Ich habs mit fucking 3 Autos in allen geschwindigkeiten die es gibt versucht.
    # Und ja, ich habe sowohl drive_control, als auch parking so modifizert das man
    # mit pub_speed die geschwindigkeit ändern kann, gut und der ganze 
    # vorwärtskram, aber das war nur copy/paste.
    # Auf jeden Fall fährt das Auto mal 30 Grad zu wenig, mal zu viel
    # machmal denkt es sich auch ich kann es mal kreuzweise und
    # fährt statt einem halbkreis 3 volle Kreise oder hört gar nicht mehr auf.
    # Nach Stunden der feinjustierung kam ich zum Schluss, dass 
    # eine Feinjustierung bei +-30 Grad nicht wirklich sinnvoll ist.
    # Ich vermute es liegt an 'odom', ich gehe einfach davon aus das, dass die
    # Kamera bestimmt postion ist.
    # Naja, es war nicht die Aufgabe, ich habs mir selbt eingebrockt und 
    # will hier auch niemanden anmekern, ich wollte nur den Frust loswerden


    speed_is_key = pub_speed
    driving_direction_pub1 = pub_back_right
    driving_direction_pub2 = pub_back_left
    gerade = pub_forward
    links = pub_forward_left
    rechts = pub_forward_right

    

    speed_is_key.publish(0.6)

    rospy.sleep(5)
    #gerade.publish(0.05)
    #rospy.sleep(5)
    rechts.publish(0.72)
    rospy.sleep(30)
    links.publish(1.2)
    rospy.sleep(30)
    gerade.publish(2.4)

    # Das hier ist nur wichtig wenn dannach noch was gemacht werden soll,
    # es setzt halt den speed zurück
    speed_is_key.publish(0.3)
    
    return

def callbackBackwardLongitudinal(request):
    rospy.loginfo(rospy.get_caller_id() + ": callbackBackwardLongitudinal, direction = " + request.direction)

    if request.direction == "left":
        driving_direction_pub1 = pub_back_left
        driving_direction_pub2 = pub_back_right
    elif request.direction == "right":
        driving_direction_pub1 = pub_back_right
        driving_direction_pub2 = pub_back_left
    elif request.direction == "baum":
        # Ja wir haben es in ne extra Funktion gepackt, fanden wir schönen
        dasBesteEinparkenEverz()
        return ParkingManeuverResponse("My job here is done")
    elif request.direction == "GodLevel":
        dasAlphaUndOmega()
        return ParkingManeuverResponse("Gern geschehen")
    else:
        return ParkingManeuverResponse(
            "ERROR: Request can only be 'left' or 'right'")

    driving_direction_pub1.publish(0.30)

#     rospy.sleep(10)
#     pub_back.publish(0.10)

    rospy.sleep(10)
    driving_direction_pub2.publish(0.30)

    rospy.sleep(10)
    pub_forward.publish(0.1)

    rospy.sleep(10)
    return ParkingManeuverResponse("FINISHED")


# --- main program ---

# In ROS, nodes are uniquely named. If two nodes with the same
# node are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our node so that multiple instances can
# run simultaneously.
rospy.init_node("simple_parking_maneuver")

# create subscribers and publishers
sub_info = rospy.Subscriber(
    "simple_drive_control/info", String, callbackDrivingControl, queue_size=10)
sub_backward_longitudinal = rospy.Service(
    "simple_parking_maneuver/backward_longitudinal",
    ParkingManeuver,
    callbackBackwardLongitudinal)

pub_back_left = rospy.Publisher(
    "simple_drive_control/backward_left",
    Float32,
    queue_size=10)
pub_back_right = rospy.Publisher(
    "simple_drive_control/backward_right",
    Float32,
    queue_size=10)
pub_back = rospy.Publisher(
    "simple_drive_control/backward",
    Float32,
    queue_size=10)
pub_forward = rospy.Publisher(
    "simple_drive_control/forward",
    Float32,
    queue_size=10)

pub_forward_left = rospy.Publisher(
    "simple_drive_control/forward_left",
    Float32,
    queue_size=10)

pub_forward_right = rospy.Publisher(
    "simple_drive_control/forward_right",
    Float32,
    queue_size=10)

pub_speed = rospy.Publisher(
    "simple_drive_control/speed",
    Float32,
    queue_size=10)


rospy.loginfo(rospy.get_caller_id() + ": started!")

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()

