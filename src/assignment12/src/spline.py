#!/usr/bin/env python

import rospy
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate

from autominy_msgs.msg import NormalizedSpeedCommand, NormalizedSteeringCommand, SteeringFeedback
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, Int32
from sensor_msgs.msg import LaserScan


data = np.load('lane1.npy')
datab = np.load('lane2.npy')
print len(data[:,0]);
lane = 1
timecounter = 0
aktuellerpunkt = Odometry()
isactiv = False

f = interpolate.CubicSpline(data[:,0],data[:,1])
f2 = interpolate.CubicSpline(data[:,0],data[:,2])

fb = interpolate.CubicSpline(datab[:,0],datab[:,1])
f2b = interpolate.CubicSpline(datab[:,0],datab[:,2])

xnew = np.arange(0, 12.8, 0.1)
ynew = f(xnew)

xnewb = np.arange(0, 14.8, 0.1)
ynewb = fb(xnewb)
#plt.plot(data[:,0], data[:,1], 'o', xnew, ynew, '-')
#plt.show()

xnew2 = np.arange(0, 12.8, 0.1)
ynew2 = f2(xnew2)

xnew2b = np.arange(0, 14.8, 0.1)
ynew2b = f2b(xnew2b)
#plt.plot(data[:,0], data[:,2], 'o', xnew2, ynew2, '-')
#plt.show()

#plt.plot(f(xnew),f2(xnew2))
#plt.show()

def findenaechsten(x,y):
    closex = 100.0
    closey = 100.0
    for i in np.arange(1.0,12.0,1.0):
            if math.sqrt((x-f(i))**2+(y-f2(i))**2)<math.sqrt((x-f(closex))**2+(y-f2(closey))**2):
                closex = i
                closey = i
            #print (i,closex,closey,f(closex),f2(closey))

    for k in np.arange(1.0,10.0,1.0):
        oldclosex = closex
        oldclosey = closey
        correctionlowx = 0.0
        correctionlowy = 0.0
        correctionhighx = 0.0
        correctionhighy = 0.0
        if closex<(3.0/k):
           correctionlowx = (3.0/k)-closex
        if closex>12.0-(3.0/k):
            correctionhighx = closex-12.0+(3.0/k)
        if closey<(3.0/k):
            correctionlowy = (3.0/k)-closey
        if closey>12.0-(3.0/k):
            correctionhighy = closey-12.0+(3.0/k)

        for i in np.arange(oldclosex-(3.0/k)+correctionlowx,oldclosex+(3.0/k)-correctionhighx,0.5/k):
                if math.sqrt((x-f(i))**2+(y-f2(i))**2)<math.sqrt((x-f(closex))**2+(y-f2(closey))**2):
                    closex = i
                    closey = i
                #print (i,closex,closey,f(closex),f2(closey),f(i),f2(i))

    #print (closex,closey)
    return closex,closey

def findenaechstenb(x,y):
    closex = 100.0
    closey = 100.0
    for i in np.arange(1.0,14.0,1.0):
            if math.sqrt((x-fb(i))**2+(y-f2b(i))**2)<math.sqrt((x-fb(closex))**2+(y-f2b(closey))**2):
                closex = i
                closey = i
            #print (i,closex,closey,f(closex),f2(closey))

    for k in np.arange(1.0,10.0,1.0):
        oldclosex = closex
        oldclosey = closey
        correctionlowx = 0.0
        correctionlowy = 0.0
        correctionhighx = 0.0
        correctionhighy = 0.0
        if closex<(3.0/k):
           correctionlowx = (3.0/k)-closex
        if closex>14.0-(3.0/k):
            correctionhighx = closex-14.0+(3.0/k)
        if closey<(3.0/k):
            correctionlowy = (3.0/k)-closey
        if closey>14.0-(3.0/k):
            correctionhighy = closey-14.0+(3.0/k)

        for i in np.arange(oldclosex-(3.0/k)+correctionlowx,oldclosex+(3.0/k)-correctionhighx,0.5/k):
                if math.sqrt((x-fb(i))**2+(y-f2b(i))**2)<math.sqrt((x-fb(closex))**2+(y-f2b(closey))**2):
                    closex = i
                    closey = i
                #print (i,closex,closey,f(closex),f2(closey),f(i),f2(i))

    #print (closex,closey)
    return closex,closey

def findenaechstenpunkt(x,y):
    a,b = findenaechsten(x,y)
    marker3 = Marker()
    marker3.header.frame_id = "map"
    marker3.ns = "road"
    marker3.id = 0;
    marker3.type = Marker.SPHERE
    marker3.action = Marker.ADD
    marker3.scale.x = 0.5
    marker3.scale.y = 0.5
    marker3.scale.z = 0.5
    marker3.pose.position.x = f(a)
    marker3.pose.position.y = f2(b)
    marker3.pose.position.z = 0.0
    marker3.pose.orientation.x = 0.0
    marker3.pose.orientation.y = 0.0
    marker3.pose.orientation.z = 0.0
    marker3.pose.orientation.w = 1.0
    marker3.color.a = 1.0
    marker3.color.r = 1.0
    marker3.color.g = 0.0
    marker3.color.b = 1.0
    pub_close.publish(marker3)
    return(f(a),f2(b))

def findenaechstenpunktb(x,y):
    a,b = findenaechstenb(x,y)
    marker3 = Marker()
    marker3.header.frame_id = "map"
    marker3.ns = "road"
    marker3.id = 0;
    marker3.type = Marker.SPHERE
    marker3.action = Marker.ADD
    marker3.scale.x = 0.5
    marker3.scale.y = 0.5
    marker3.scale.z = 0.5
    marker3.pose.position.x = fb(a)
    marker3.pose.position.y = f2b(b)
    marker3.pose.position.z = 0.0
    marker3.pose.orientation.x = 0.0
    marker3.pose.orientation.y = 0.0
    marker3.pose.orientation.z = 0.0
    marker3.pose.orientation.w = 1.0
    marker3.color.a = 1.0
    marker3.color.r = 1.0
    marker3.color.g = 0.0
    marker3.color.b = 1.0
    pub_close.publish(marker3)
    return(fb(a),f2b(b))


visiondistance = 0.9
def lookahead(x,y):
    a,b = findenaechsten(x,y)
    marker4 = Marker()
    marker4.header.frame_id = "map"
    marker4.ns = "road"
    marker4.id = 0;
    marker4.type = Marker.SPHERE
    marker4.action = Marker.ADD
    marker4.scale.x = 0.5
    marker4.scale.y = 0.5
    marker4.scale.z = 0.5
    marker4.pose.position.x = f((a+visiondistance)%12.6)
    marker4.pose.position.y = f2((b+visiondistance)%12.6)
    marker4.pose.position.z = 0.0
    marker4.pose.orientation.x = 0.0
    marker4.pose.orientation.y = 0.0
    marker4.pose.orientation.z = 0.0
    marker4.pose.orientation.w = 1.0
    marker4.color.a = 1.0
    marker4.color.r = 1.0
    marker4.color.g = 0.0
    marker4.color.b = 0.0
    pub_lookahead.publish(marker4)
    return f((a+visiondistance)%12.8),f2((b+visiondistance)%12.8)

def lookaheadb(x,y):
    a,b = findenaechstenb(x,y)
    marker4 = Marker()
    marker4.header.frame_id = "map"
    marker4.ns = "road"
    marker4.id = 0;
    marker4.type = Marker.SPHERE
    marker4.action = Marker.ADD
    marker4.scale.x = 0.5
    marker4.scale.y = 0.5
    marker4.scale.z = 0.5
    marker4.pose.position.x = fb((a+visiondistance)%14.6)
    marker4.pose.position.y = f2b((b+visiondistance)%14.6)
    marker4.pose.position.z = 0.0
    marker4.pose.orientation.x = 0.0
    marker4.pose.orientation.y = 0.0
    marker4.pose.orientation.z = 0.0
    marker4.pose.orientation.w = 1.0
    marker4.color.a = 1.0
    marker4.color.r = 1.0
    marker4.color.g = 0.0
    marker4.color.b = 0.0
    pub_lookahead.publish(marker4)
    return fb((a+visiondistance)%14.8),f2b((b+visiondistance)%14.8)

def fangePunkt(punkt):
    global lane, timecounter, aktuellerpunkt
    aktuellerpunkt = punkt
    #print(timecounter, lane)
    if(timecounter > 200):
        #pub_lane.publish(3-lane)
        timecounter = 0
    #timecounter = timecounter + 1
    if(lane==1):
        marker2 = Marker()
        marker2.header.frame_id = "map"
        marker2.ns = "road"
        marker2.id = 0;
        marker2.type = Marker.SPHERE
        marker2.action = Marker.ADD
        marker2.scale.x = 0.5
        marker2.scale.y = 0.5
        marker2.scale.z = 0.5
        marker2.pose.position.x = punkt.pose.pose.position.x
        marker2.pose.position.y = punkt.pose.pose.position.y
        marker2.pose.position.z = 0.0
        marker2.pose.orientation.x = 0.0
        marker2.pose.orientation.y = 0.0
        marker2.pose.orientation.z = 0.0
        marker2.pose.orientation.w = 1.0
        marker2.color.a = 1.0
        marker2.color.r = 0.0
        marker2.color.g = 0.0
        marker2.color.b = 1.0
        pub_clicked.publish(marker2)
        lookahead(punkt.pose.pose.position.x, punkt.pose.pose.position.y)
        findenaechstenpunkt(punkt.pose.pose.position.x, punkt.pose.pose.position.y)
    else:
        marker2 = Marker()
        marker2.header.frame_id = "map"
        marker2.ns = "road"
        marker2.id = 0;
        marker2.type = Marker.SPHERE
        marker2.action = Marker.ADD
        marker2.scale.x = 0.5
        marker2.scale.y = 0.5
        marker2.scale.z = 0.5
        marker2.pose.position.x = punkt.pose.pose.position.x
        marker2.pose.position.y = punkt.pose.pose.position.y
        marker2.pose.position.z = 0.0
        marker2.pose.orientation.x = 0.0
        marker2.pose.orientation.y = 0.0
        marker2.pose.orientation.z = 0.0
        marker2.pose.orientation.w = 1.0
        marker2.color.a = 1.0
        marker2.color.r = 0.0
        marker2.color.g = 0.0
        marker2.color.b = 1.0
        pub_clicked.publish(marker2)
        lookaheadb(punkt.pose.pose.position.x, punkt.pose.pose.position.y)
        findenaechstenpunktb(punkt.pose.pose.position.x, punkt.pose.pose.position.y)

def fangeLaser(laser):
    global isactiv
    if(True):
        isactiv = True
        #print("the laser has been fired", len(laser.ranges))
        j = 0.0
        laserpunkte = []
        for k in np.arange(0,360,1):
            #print(k,k*0.017453)
            i = laser.ranges[k]
            k = k+180
            if (abs(i)<2.0):
                hilfspunkt = Point()
                hilfspunkt.x = aktuellerpunkt.pose.pose.position.x + i*math.cos(k*0.017453)
                hilfspunkt.y = aktuellerpunkt.pose.pose.position.y + i*math.sin(k*0.017453)
                hilfspunkt.z = 0.0
                pruefeSpur(hilfspunkt, k)
                lasermarker2 = Marker()
                lasermarker2.header.frame_id = "map"
                lasermarker2.id = 0;
                lasermarker2.ns = "laser"+str(j)
                lasermarker2.type = Marker.SPHERE
                lasermarker2.action = Marker.ADD
                lasermarker2.scale.x = 0.1
                lasermarker2.scale.z = 0.1
                lasermarker2.scale.y = 0.1
                lasermarker2.pose.position.x = aktuellerpunkt.pose.pose.position.x + i*math.cos(k*0.017453)
                lasermarker2.pose.position.y = aktuellerpunkt.pose.pose.position.y + i*math.sin(k*0.017453)
                lasermarker2.pose.position.z = 0.0
                lasermarker2.pose.orientation.x = 0.0
                lasermarker2.pose.orientation.y = 0.0
                lasermarker2.pose.orientation.z = 0.0
                lasermarker2.pose.orientation.w = 1.0
                lasermarker2.color.a = 1.0
                lasermarker2.color.r = 0.0
                lasermarker2.color.g = 1.0
                lasermarker2.color.b = 0.0
                pub_lasersurround.publish(lasermarker2)
                laserpunkte.append(hilfspunkt)

            else:
                lasermarker2 = Marker()
                lasermarker2.header.frame_id = "map"
                lasermarker2.id = 0;
                lasermarker2.ns = "laser"+str(j)
                lasermarker2.type = Marker.SPHERE
                lasermarker2.action = Marker.ADD
                lasermarker2.scale.x = 0.1
                lasermarker2.scale.z = 0.1
                lasermarker2.scale.y = 0.1
                lasermarker2.pose.position.x = aktuellerpunkt.pose.pose.position.x + 2.0*math.cos(k*0.017453)
                lasermarker2.pose.position.y = aktuellerpunkt.pose.pose.position.y + 2.0*math.sin(k*0.017453)
                lasermarker2.pose.position.z = 0.0
                lasermarker2.pose.orientation.x = 0.0
                lasermarker2.pose.orientation.y = 0.0
                lasermarker2.pose.orientation.z = 0.0
                lasermarker2.pose.orientation.w = 1.0
                lasermarker2.color.a = 1.0
                lasermarker2.color.r = 1.0
                lasermarker2.color.g = 1.0
                lasermarker2.color.b = 1.0
                pub_lasersurround.publish(lasermarker2)

            j = j+1

        laserpunkte.append(punkte[0])
        lasermarker = Marker()
        lasermarker.header.frame_id = "map"
        lasermarker.ns = "laser"
        lasermarker.id = 0;
        lasermarker.type = Marker.LINE_STRIP
        lasermarker.action = Marker.ADD
        lasermarker.scale.x = 0.3
        lasermarker.scale.y = 0.3
        lasermarker.scale.z = 0.3
        lasermarker.pose.position.x = 0.0
        lasermarker.pose.position.y = 0.0
        lasermarker.pose.position.y = 0.0
        lasermarker.pose.orientation.x = 0.0
        lasermarker.pose.orientation.z = 0.0
        lasermarker.pose.orientation.y = 0.0
        lasermarker.pose.orientation.w = 1.0
        lasermarker.points = laserpunkte
        lasermarker.color.a = 1.0
        lasermarker.color.g = 1.0
        lasermarker.color.r = 1.0
        lasermarker.color.b = 0.0
        #pub_lasersurround.publish(lasermarker)
    isactiv = False

def pruefeSpur(lanepunkt,k):
    global aktuellerpunkt, lane
    a,b = findenaechstenpunkt(lanepunkt.x, lanepunkt.y) #findet den nächsten punkt auf der aueßeren spur
    c,d = findenaechstenpunktb(lanepunkt.x, lanepunkt.y) #findet den nächsten punkt auf der inneren spur
    distancea = math.sqrt((lanepunkt.x-a)**2+(lanepunkt.y-b)**2) #berechnet wie weit der punkt von der Spur entfehrt ist
    distanceb = math.sqrt((lanepunkt.x-c)**2+(lanepunkt.y-d)**2) #berechnet wie weit der punkt von der Spur entfehrt ist
    print(distancea, distanceb)
    if(((distancea<0.3) or (distanceb<0.3)) and ((k>0.2) or (k>0.2))): #wenn der gegenstand nah genug ist wird ausgewichen, das auto scannt sein hinterteil, daher muss es hier einen mindestabstand haben
        if(distancea<distanceb):
            lane = 2
        else:
            lane = 1


def changelane(i):
    global lane
    lane = i.data
    #print("new lane = ", i)

punkte = []
for h in np.arange(0.1,12.7,0.2):
    hilfspunkt = Point()
    hilfspunkt.x = f(h)
    hilfspunkt.y = f2(h)
    hilfspunkt.z = 0.0
    punkte.append(hilfspunkt)
punkte.append(punkte[0])

marker = Marker()
marker.header.frame_id = "map"
marker.ns = "road"
marker.id = 0;
marker.type = Marker.LINE_STRIP
marker.action = Marker.ADD
marker.scale.x = 0.3
marker.scale.y = 0.3
marker.scale.z = 0.3
marker.pose.position.x = 0.0
marker.pose.position.y = 0.0
marker.pose.position.y = 0.0
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0
marker.points = punkte
marker.color.a = 1.0
marker.color.r = 1.0
marker.color.g = 1.0
marker.color.b = 0.0

punkteb = []
for h in np.arange(0.1,14.7,0.2):
    hilfspunkt = Point()
    hilfspunkt.x = fb(h)
    hilfspunkt.y = f2b(h)
    hilfspunkt.z = 0.0
    punkteb.append(hilfspunkt)
punkteb.append(punkteb[0])

markerb = Marker()
markerb.header.frame_id = "map"
markerb.ns = "road"
markerb.id = 0;
markerb.type = Marker.LINE_STRIP
markerb.action = Marker.ADD
markerb.scale.x = 0.3
markerb.scale.y = 0.3
markerb.scale.z = 0.3
markerb.pose.position.x = 0.0
markerb.pose.position.y = 0.0
markerb.pose.position.y = 0.0
markerb.pose.orientation.x = 0.0
markerb.pose.orientation.y = 0.0
markerb.pose.orientation.z = 0.0
markerb.pose.orientation.w = 1.0
markerb.points = punkteb
markerb.color.a = 1.0
markerb.color.r = 1.0
markerb.color.g = 1.0
markerb.color.b = 0.0
rospy.init_node("spline")



pub_road = rospy.Publisher("/spline/road", Marker, queue_size=10)
pub_road2 = rospy.Publisher("/spline/road2", Marker, queue_size=10)
pub_lasersurround = rospy.Publisher("/spline/laser", Marker, queue_size=1)
sub_point = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, fangePunkt, queue_size=10)
sub_lane = rospy.Subscriber("/GiveMeALaneNumberOrWeWillAllDieBecauseIDontKnoeWhereToDrive", Int32, changelane, queue_size=10)
sub_laser = rospy.Subscriber("/sensors/rplidar/scan", LaserScan, fangeLaser, queue_size=1)
pub_lane = rospy.Publisher("/GiveMeALaneNumberOrWeWillAllDieBecauseIDontKnoeWhereToDrive", Int32, queue_size=10)
pub_lookahead = rospy.Publisher("/spline/lookahead", Marker, queue_size=10)
pub_close = rospy.Publisher("/spline/close", Marker, queue_size=10)
pub_clicked = rospy.Publisher("/spline/clicked", Marker, queue_size=10)



def main():
    print ("wait for it")
    rospy.sleep(2)
    pub_road.publish(marker)
    pub_road2.publish(markerb)
    print("ready for action")


if __name__ == '__main__':
    main()


rospy.spin()
