#!/usr/bin/env python
import roslib
roslib.load_manifest('site_violation_detection')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from pylab import *
from datetime import datetime
import time
import os
import os
import glob
from os.path import expanduser
home = expanduser("~")

lastTimeImageAnalysed   = datetime.now()
lastTimeImagePublished  = datetime.now()
lastTimeDImageSaved     = datetime.now()
lastTimeHImageSaved     = datetime.now()

imagesFolder = ''
imageCounter = 0
#How often in seconds to analyse streamed images
analysisTime = 5
#How often in seconds to publish analysis results
publishingTime = 1
# Saving time
imageSavingTime = 10

class image_converter:
  global imagesFolder
  global violationImagesFolder
  global analysisTime
  global publishingTime
  
  def __init__(self):
    global imagesFolder
    global violationImagesFolder
    rospy.init_node('site_violation_detection', anonymous=True)
    self.image_pub = rospy.Publisher("/site_violation_detection/image_raw",Image, queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/web_cam/image_raw",Image,self.callback)
    
    #Create TimeStamped Image folder
    ts = time.time()
    imagesFolder = home + '/workspace/webcam_images/' + datetime.fromtimestamp(ts).strftime('%Y_%m_%d_%H_%M_%S') + '/'
    d = os.path.dirname(imagesFolder)
    if not os.path.exists(d):
        os.makedirs(d)
    
    violationImagesFolder = '/var/www/html/wordpress/wp-content/themes/violation_images/'
    
    #clear old violation images (the webapp should display only recent images)    
    files = glob.glob( violationImagesFolder + '*')
    for f in files:
      os.remove(f)      
    d = os.path.dirname(violationImagesFolder)    
    if not os.path.exists(d):
        os.makedirs(d)
        
  def callback(self,data):
    global imageCounter
    global lastTimeImageAnalysed
    global lastTimeImagePublished
    global lastTimeDImageSaved
    global lastTimeHImageSaved
    global imageSavingTime
    global imagesFolder
    global analysisTime
    global publishingTime
    
    currentTime = datetime.now()
    timeDiff = (currentTime - lastTimeImageAnalysed).seconds
    if  timeDiff< analysisTime :
      return
    lastTimeImageAnalysed = currentTime
    try:
      cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e
    
    violationFound = False
    
    image2Analyse = cvImage.copy()
    #cnt = cvImage.copy()
    (rows,cols,channels) = image2Analyse.shape
    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    
    gray = cv2.cvtColor(image2Analyse,cv2.COLOR_BGR2GRAY) # convert to gray scale
    #eroded = cv2.erode(gray,element) # erode the image
    #edg = cv2.Canny(eroded,50,50) # detect edges canny
    filtered = cv2.adaptiveThreshold(gray,  255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 21, 1)
    #cv2.imshow('Filtered',filtered)
    image, contours, hierarchy = cv2.findContours(filtered, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # find contours
    #cv2.drawContours(cnt, contours, -1, (0,255,0), 3)
    #cv2.imshow('Contours',cnt)
    
    c = max(contours, key = cv2.contourArea) # find maximum contour
    #x,y,w,h = cv2.boundingRect(c) # find the rectangle that surrounds the contour
    #cv2.rectangle(image2Analyse, (x,y),(x+w,y+h), (0,255,0), 3)# Draw the rectangle that surrounds the maximum contour
    #cv2.drawContours(image2Analyse, c, -1, (255,255,0), 3)
    k   = cv2.minAreaRect(c)
    box = cv2.boxPoints(k)
    box = np.int0(box)
    cv2.drawContours(image2Analyse,[box],0,(0,255,0),3)
    width = min(k[1])
    length = max(k[1])
    #cv2.imshow('Live',cvImage)
    if length <= 1.1*width or width >= 0.9*length:
      cv2.putText(image2Analyse,"Violation Detected", (2, rows / 2), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255),2)
      cv2.putText(image2Analyse,"Exceeded Allowed Dimensions", (50, rows / 2 + 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255),2)
      ts = time.time()
      cv2.putText(image2Analyse,datetime.fromtimestamp(ts).strftime('%Y/%m/%d %H:%M:%S'), (10, rows - 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),1)
      #cv2.imshow('Violation 1',image2Analyse)
      #cv2.waitKey(5)
      ViolationName_d = violationImagesFolder + 'ViolationImage_dimension' + ("%03d" % imageCounter) + '.png'
      currentTime = datetime.now()
      timeDiff    = (currentTime - lastTimeDImageSaved).seconds
      #Save images only at pre-defined interval
      if  timeDiff > imageSavingTime :
        lastTimeDImageSaved = currentTime
        cv2.imwrite(ViolationName_d, image2Analyse) 
      violationFound = True
    #else:
    #  cv2.imshow('Violation 1',image2Analyse)
      #cv2.waitKey(5)
    #cv2.putText(image2Analyse,"violation detected", (50, rows / 2 + 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255),2)
    # convert to hsv color space
    
    im2 = cv2.cvtColor(cvImage,cv2.COLOR_BGR2HSV)
    # define the list of boundaries for blue color
    lower = np.array([110, 150, 150])
    upper = np.array([130, 255, 255])
    # find the colors within the specified boundaries and apply the mask
    mask   = cv2.inRange(im2, lower, upper)
    output = cv2.bitwise_and(im2, im2, mask = mask)
    if mask.any() != 0:
      image2Analyse = cvImage.copy()
      cv2.putText(image2Analyse,"Human Detected", (50, rows / 2), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255),2)
      cv2.putText(image2Analyse,"(Outside working hours)", (125, rows / 2 + 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2)
      ts = time.time()
      cv2.putText(image2Analyse,datetime.fromtimestamp(ts).strftime('%Y/%m/%d %H:%M:%S'), (10, rows - 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),1)      
      #cv2.imshow('Violation 2',image2Analyse)
      ViolationName_h = violationImagesFolder + 'ViolationImage_human' + ("%03d" % imageCounter) + '.png'
      currentTime = datetime.now()
      timeDiff    = (currentTime - lastTimeHImageSaved).seconds
      #Save images only at pre-defined interval
      if  timeDiff > imageSavingTime :
        lastTimeHImageSaved = currentTime
        cv2.imwrite(ViolationName_h, image2Analyse)
      violationFound = True
      #else:
      #  cv2.imshow('Violation 2',im2)

    if not violationFound:
      image2Analyse = cvImage.copy()
      ts = time.time()
      cv2.putText(image2Analyse,datetime.fromtimestamp(ts).strftime('%Y/%m/%d %H:%M:%S'), (10, rows - 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),1)      
      cv2.putText(image2Analyse,"Detecting Violations", (5, rows / 2), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,255,0),2)
    
    # Save all raw images for later analysis
    imageName = imagesFolder + 'analysedImage' + ("%03d" % imageCounter) + '.png'
    cv2.imwrite(imageName, cvImage)   
    
    #Publish only at the requested Frequency, we don't want to use a lot of bandwith    
    currentTime = datetime.now()
    timeDiff = (currentTime - lastTimeImagePublished).seconds    
    if  timeDiff > publishingTime :
      lastTimeImagePublished = currentTime
      try:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(image2Analyse, "bgr8"))
      except CvBridgeError, e:
        print e
    
    imageCounter = imageCounter + 1
    
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)
