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

lastTimeImageAnalysed = datetime.now()
imagesFolder = ''
imageCounter = 0
class image_converter:
  global imagesFolder
  global violationImagesFolder
  
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
    violationImagesFolder = home + '/workspace/violation_images/'
    
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
    global imagesFolder
    currentTime = datetime.now()
    timeDiff = (currentTime - lastTimeImageAnalysed).seconds
    if  timeDiff< 1.0 :
      return
    lastTimeImageAnalysed = currentTime
    try:
      cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e
    im3 = cvImage.copy()
    cnt = cvImage.copy()
    (rows,cols,channels) = im3.shape
    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    
    gray = cv2.cvtColor(im3,cv2.COLOR_BGR2GRAY) # convert to gray scale
    #eroded = cv2.erode(gray,element) # erode the image
    #edg = cv2.Canny(eroded,50,50) # detect edges canny
    filtered = cv2.adaptiveThreshold(gray,  255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 21, 1)
    #cv2.imshow('Filtered',filtered)
    contours, hierarchy = cv2.findContours(filtered, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # find contours
    #cv2.drawContours(cnt, contours, -1, (0,255,0), 3)
    #cv2.imshow('Contours',cnt)
    
    c = max(contours, key = cv2.contourArea) # find maximum contour
    x,y,w,h = cv2.boundingRect(c) # find the rectangle that surrounds the contour
    cv2.rectangle(im3, (x,y),(x+w,y+h), (0,255,0), 3)# Draw the rectangle that surrounds the maximum contour
    #cv2.drawContours(im3, c, -1, (255,255,0), 3)
    k=cv2.minAreaRect(c)
    width = min(k[1])
    length = max(k[1])
    #cv2.imshow('Live',cvImage)
    if length <= 1.1*width or width >= 0.9*length:
      cv2.putText(im3,"Violation Detected", (2, rows / 2), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255),2)
      cv2.putText(im3,"Exceeded Allowed Dimensions", (50, rows / 2 + 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255),2)
      #cv2.imshow('Violation 1',im3)
      #cv2.waitKey(5)
      ViolationName_d = violationImagesFolder + 'ViolationImage_dimension' + ("%03d" % imageCounter) + '.png'
      cv2.imwrite(ViolationName_d, im3) 
    #else:
    #  cv2.imshow('Violation 1',im3)
      #cv2.waitKey(5)
    #cv2.putText(im3,"violation detected", (50, rows / 2 + 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255),2)
    # convert to hsv color space
    image2 = cv2.cvtColor(cvImage,cv2.COLOR_BGR2HSV)
    # define the list of boundaries for blue color
    lower = np.array([110, 150, 150])
    upper = np.array([130, 255, 255])
    # find the colors within the specified boundaries and apply the mask
    mask   = cv2.inRange(image2, lower, upper)
    output = cv2.bitwise_and(image2, image2, mask = mask)
    im2 = cvImage.copy()
    # show the images
    if mask.any() != 0:
      cv2.putText(im2,"Human presence detected", (50, rows / 2 + 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255),2)
      #cv2.imshow('Violation 2',im2)
      ViolationName_h = violationImagesFolder + 'ViolationImage_human' + ("%03d" % imageCounter) + '.png'
      cv2.imwrite(ViolationName_h, im2) 
    #else:
    #  cv2.imshow('Violation 2',im2)

    imageName = imagesFolder + 'analysedImage' + ("%03d" % imageCounter) + '.png'
    cv2.imwrite(imageName, cvImage)   
    
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(im3, "bgr8"))
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
