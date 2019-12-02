#!/usr/bin/env python

import rospy
import math
import cv2
import numpy as np
import random


from std_msgs.msg import Float32, Header
from sensor_msgs.msg import CameraInfo, Image, CompressedImage

def toeteDasFalscheWeiss(bild):
    for i in range(len(bild)-2):
        for j in range(len(bild[0])-2):
              if (i<170 or i>300 or j<190 or j>460):
                   bild[i,j] = (0,0,0)
    return bild


weiterleitung = {}

def twoPassKacke(bild):
    counter = 1;
    better = np.zeros([len(bild),len(bild[0])], dtype=np.uint32)
    for i in range(len(bild)-2):
        for j in range(len(bild[0])-2):
            if ((bild[i][j][0] > 250)):
                if ((better[i-1,j] != 0) or (better[i,j-1] != 0)):
                    if ((better[i-1,j] != 0) and (better[i,j-1] != 0)):
                        better[i,j] = min(better[i-1,j], better[i,j-1])
                        if (better[i-1,j] != better[i,j-1]):
                            weiterleitung[max(better[i-1,j], better[i,j-1])] = min(better[i-1,j], better[i,j-1])
                    else:
                        better[i,j] = max(better[i-1,j], better[i,j-1])
                else:
                    better[i,j] = counter
                    counter = counter + 1

    return better

def weiterZahl(a):
    if a in weiterleitung:
        return weiterZahl(weiterleitung[a])
    return a

def malDieTwoPassKacke(better):
    colors = [(255,255,255),(0,255,0),(0,0,255),(255,0,255),(0,255,255),(255,255,0)]
    new = np.zeros([len(better),len(better[0]),3], dtype=np.uint8)
    for i in range(len(better)):
        for j in range(len(better[0])):
             if (weiterZahl(better[i,j]) != 0):
                 new[i,j] = colors[(weiterZahl(better[i,j])%len(colors))]
             else:
                 new[i,j] = (0,0,0)
    return new


Anzahl = {}

def findeZweiPunkte(TPList):
    for k in range(len(weiterleitung)):
        Anzahl[k] = 0
    pixel = 0
    pixel11 = (0,0)
    pixel12 = (0,0)
    pixel21 = (0,0)
    pixel22 = (0,0)
    for i in range(len(TPList)):
        for j in range(len(TPList[0])):
            #if (TPList[i,j]!=0):
                pixel = pixel+1
                Anzahl[weiterZahl(TPList[i,j])] = Anzahl[weiterZahl(TPList[i,j])]+1

    #2 random points from first lane
    counter1 = 0
    counter2 = 0
    for i in range(len(TPList)):
        for j in range(len(TPList[0])):
            if (pixel11==(0,0) and weiterZahl(TPList[i,j])==1):
                #print counter1,((1/(float(Anzahl[1])-float(counter1))))
                if (random.random()<((1/(float(Anzahl[1])-float(counter1))))):
                    pixel11 = (i,j)
                counter1 = counter1+1
    for i in range(len(TPList)):
        for j in range(len(TPList[0])):
            if (pixel12==(0,0) and weiterZahl(TPList[i,j])==1):
                #print counter2,((1/(float(Anzahl[1])-float(counter2))))
                if (random.random()<((1/(float(Anzahl[1])-float(counter2))))):
                    pixel12 = (i,j)
                counter2 = counter2+1


    #2 random points from first lane
    counter1 = 0
    counter2 = 0
    for i in range(len(TPList)):
        for j in range(len(TPList[0])):
            if (pixel21==(0,0) and weiterZahl(TPList[i,j])==2):
                #print counter1,((1/(float(Anzahl[1])-float(counter1))))
                if (random.random()<((1/(float(Anzahl[2])-float(counter1))))):
                    pixel21 = (i,j)
                counter1 = counter1+1
    for i in range(len(TPList)):
        for j in range(len(TPList[0])):
            if (pixel22==(0,0) and weiterZahl(TPList[i,j])==2):
                #print counter2,((1/(float(Anzahl[2])-float(counter2))))
                if (random.random()<((1/(float(Anzahl[2])-float(counter2))))):
                    pixel22 = (i,j)
                counter2 = counter2+1

    print pixel11, pixel12, pixel21, pixel22
    return pixel11, pixel12, pixel21, pixel22

fertig = 0

def cameraBild(Bild):
    #print Bild
    global fertig
    if (fertig == 0):
        f = open("raw_car.jpeg","w+")
        f.write(Bild.data)
        f.close()
        bearbeiteBild()
    fertig = 1
    return

perfekteLinie1 = [(0,0),(0,0),0]
perfekteLinie2 = [(0,0),(0,0),0]


def pruefeLinie(TPList, (x,y), (x2,y2), lane):
    global perfekteLinie1, perfekteLinie2
    inliner = 0
    steigung = float(y2-y)/float(x2-x)
    for i in range(len(TPList)):
        for j in range(len(TPList[0])):
            if (weiterZahl(TPList[i,j])==lane):
                if abs((steigung*(i-x)+y)-j)<10:
                    inliner = inliner+1
    if (lane==1):
        if (perfekteLinie1[2]<inliner):
            perfekteLinie1 = [(y,x),(y2,x2),inliner]
    if (lane==2):
        if (perfekteLinie2[2]<inliner):
            perfekteLinie2 = [(y,x),(y2,x2),inliner]
    print inliner



def bearbeiteBild():
    global perfekteLinie1, perfekteLinie2
    pixel11, pixel12, pixel21, pixel22 = (0,0),(0,0),(0,0),(0,0)
    img = cv2.imread('raw_car.jpeg',0)
    ret, img = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
    cv2.imwrite("binary.jpeg",img)
    img2 = cv2.imread("binary.jpeg")
    img2 = toeteDasFalscheWeiss(img2)
    TPimg2 = twoPassKacke(img2)
    img2 = malDieTwoPassKacke(TPimg2)
    cv2.imwrite("2passBild.jpeg",img2)
    for i in range(3):
        pixel11, pixel12, pixel21, pixel22 = findeZweiPunkte(TPimg2)
        pruefeLinie(TPimg2, pixel11, pixel12,1)
        pruefeLinie(TPimg2, pixel21, pixel22,2)
    print perfekteLinie1
    print perfekteLinie2

    img3 = cv2.imread('raw_car.jpeg',0)
    cv2.line(img3,perfekteLinie1[0],perfekteLinie1[1],(0,0,0),2)
    cv2.line(img3,perfekteLinie2[0],perfekteLinie2[1],(0,0,0),2)
    cv2.imwrite("done.jpeg",img3)
    #header = Header()
    #NewCompressedImage = np.zeros((len(img3[0])*len(img3)), dtype=np.uint8)
    #img4 = cv2.cvtColor(img3,cv2.COLOR_BGR2GRAY)
    #for i in range(len(img4)):
    #    for j in range(len(img4[0])):
    #        NewCompressedImage[i+(j*len(img4))] = img4[i,j]
    #print NewCompressedImage
    #image_pub.publish(header,"jpeg",NewCompressedImage)
    (x1,y1) = perfekteLinie1[0]
    (x12,y12) = perfekteLinie1[1]
    math1_pub.publish(y1-(x2*(float(y12-y1)/float(x12-x1))))
    math12_pub.publish(float(y12-y1)/float(x12-x1))

    (x2,y2) = perfekteLinie2[0]
    (x22,y22) = perfekteLinie2[1]
    math2_pub.publish(y2-(x2*(float(y22-y2)/float(x22-x2))))
    math22_pub.publish(float(y22-y2)/float(x22-x2))
    #rospy.signal_shutdown("I am done")


rospy.init_node("ALane")

math1_pub = rospy.Publisher("ALane/Offset", Float32, queue_size=10)
math12_pub = rospy.Publisher("ALane/Steigung", Float32, queue_size=10)
math2_pub = rospy.Publisher("ALane/Offset", Float32, queue_size=10)
math22_pub = rospy.Publisher("ALane/Steigung", Float32, queue_size=10)

#image_pub = rospy.Publisher("Alane/image", CompressedImage, queue_size=10)
camera_image_sub = rospy.Subscriber("sensors/camera/infra1/image_rect_raw/compressed", CompressedImage, cameraBild, queue_size=10)


def main():
    return


if __name__ == '__main__':
    main()

rospy.spin()
