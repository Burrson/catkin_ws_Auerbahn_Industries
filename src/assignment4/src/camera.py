#!/usr/bin/env python


import rospy
from sensor_msgs.msg import CameraInfo, Image, CompressedImage
import math
import cv2
import numpy as np
from matplotlib import pyplot as plt




def toeteDasFalscheWeiss(bild):
    for i in range(len(bild)-2):
        for j in range(len(bild[0])-2):
              if (i<200 or i>350 or j<190 or j>460):
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

boundingedgelowx = {}
boundingedgehighx = {}
boundingedgelowy = {}
boundingedgehighy = {}

def boundingbox(objekte):
    for i in range(10000):
        boundingedgelowx[i-1] = 4000
        boundingedgelowy[i-1] = 4000
        boundingedgehighx[i-1] = 0
        boundingedgehighy[i-1] = 0
    for i in range(len(objekte)-2):
        for j in range(len(objekte[0])-2):
            if (i<boundingedgelowx[weiterZahl(objekte[i,j])]):
                boundingedgelowx[weiterZahl(objekte[i,j])] = i-1
            if (i>boundingedgehighx[weiterZahl(objekte[i,j])]):
                boundingedgehighx[weiterZahl(objekte[i,j])] = i+1
            if (j<boundingedgelowy[weiterZahl(objekte[i,j])]):
                boundingedgelowy[weiterZahl(objekte[i,j])] = j-1
            if (j>boundingedgehighy[weiterZahl(objekte[i,j])]):
                boundingedgehighy[weiterZahl(objekte[i,j])] = j+1

def drawline(bild):
    for i in range(9999):
        if ((boundingedgelowx[i]<len(bild)) and (boundingedgelowy[i]<len(bild[0])) and (i>0)):
            for j in range(len(bild)):
                if (j<boundingedgehighx[i] and j>boundingedgelowx[i]):
                    bild[j,boundingedgelowy[i]-1] = (255,0,0)
                    bild[j,boundingedgelowy[i]] = (255,0,0)
                    bild[j,boundingedgelowy[i]+1] = (255,0,0)
                if (j<boundingedgehighx[i] and j>boundingedgelowx[i]):
                    bild[j,boundingedgehighy[i]-1] = (255,0,0)
                    bild[j,boundingedgehighy[i]] = (255,0,0)
                    bild[j,boundingedgehighy[i]+1] = (255,0,0)

            for k in range(len(bild[0])):
                if (k<boundingedgehighy[i] and k>boundingedgelowy[i]):
                    bild[boundingedgelowx[i]-1,k] = (255,0,0)
                    bild[boundingedgelowx[i],k] = (255,0,0)
                    bild[boundingedgelowx[i]+1,k] = (255,0,0)
                if (k<boundingedgehighy[i] and k>boundingedgelowy[i]):
                    bild[boundingedgehighx[i]-1,k] = (255,0,0)
                    bild[boundingedgehighx[i],k] = (255,0,0)
                    bild[boundingedgehighx[i]+1,k] = (255,0,0)


    return bild

mittelPunkte = {}

def findeMittelPunkte(bild):
    zaeler = 0
    for i in range(100):
        if ((boundingedgelowx[i]<len(bild)) and (boundingedgelowy[i]<len(bild[0])) and (i>0)):
            mittelPunkte[zaeler] = ((boundingedgelowx[i]+(boundingedgehighx[i]-boundingedgelowx[i])/2),(boundingedgelowy[i]+(boundingedgehighy[i]-boundingedgelowy[i])/2))
            zaeler = zaeler + 1


def camerainfo (info):
   print "intrinsic prameters:\n",info.K[0],info.K[4],info.K[2],info.K[5]
   print "distortion coefficients:\n",info.K[0],info.K[1],info.K[2],info.K[3],info.K[4]
   rospy.signal_shutdown("I am done")

def cameraBild (Bild):
   f = open("raw_car.jpeg","w+")
   f.write(Bild.data)
   f.close()
   img = cv2.imread('raw_car.jpeg',0)
   ret, img = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
   cv2.imwrite("test.jpeg",img)
   img2 = cv2.imread("test.jpeg")
   img2 = toeteDasFalscheWeiss(img2)
   TPimg2 = twoPassKacke(img2)
   img2 = malDieTwoPassKacke(TPimg2)
   cv2.imwrite("test2.jpeg",img2)
   img3 = cv2.imread("test.jpeg")
   boundingbox(TPimg2)
   img3 = drawline(img3)
   findeMittelPunkte(img3)
   cv2.imwrite("boxed_img.jpeg",img3)
   print mittelPunkte
   rospy.signal_shutdown("I am done")


rospy.init_node("print_info")


#camera_info_sub = rospy.Subscriber("sensors/camera/infra1/camera_info", CameraInfo, camerainfo, queue_size=10)

camera_image_sub = rospy.Subscriber("sensors/camera/infra1/image_rect_raw/compressed", CompressedImage, cameraBild, queue_size=10)

rospy.spin()
