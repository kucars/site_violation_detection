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
class image_converter:

  def __init__(self):
    rospy.init_node('site_violation_detection', anonymous=True)
    self.image_pub = rospy.Publisher("/site_violation_detection/image_raw",Image, queue_size=10)
    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/web_cam/image_raw",Image,self.callback)
    
  def callback(self,data):
    try:
      cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    (rows,cols,channels) = cvImage.shape
    # convert to hsv color space
    image2 = cv2.cvtColor(cvImage,cv2.COLOR_BGR2HSV)
    # define the list of boundaries for blue color
    lower = np.array([110, 50, 50])
    upper = np.array([130, 255, 255])
	# find the colors within the specified boundaries and apply
	# the mask
    mask = cv2.inRange(image2, lower, upper)
    output = cv2.bitwise_and(image2, image2, mask = mask)
    im3 = cvImage.copy()
	# show the images
    if mask.any() != 0:
      cv2.putText(im3,"violation detected", (50, rows / 2 + 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255),2)
      #cv2.imshow('frame_blue',im3)
      #cv2.imshow('mask_blue',mask)
    #else:
      #cv2.imshow('frame_blue',cvImage)
    im = cvImage.copy()
    # convert the image to grayscale
    gray = cv2.cvtColor(im.copy(), cv2.COLOR_BGR2GRAY)
    th3 = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,15,11)
    edged = cv2.Canny(th3, 35, 100)
    # find the contours in the edged image and keep the largest one;
    (cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    c = max(cnts, key = cv2.contourArea)
    # compute the bounding box of the of the paper region and return it
    k=cv2.minAreaRect(c)
    width = min(k[1])
    length = max(k[1])
    area = width*length 
    x,y,w,h = cv2.boundingRect(c)
    cv2.rectangle(im, (x,y),(x+w,y+h), (0,0,255), 3)
    if length <= 1.1*width or width >= 0.9*length:
      cv2.putText(im,"violation detected", (50, rows / 2 + 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255),2)
    #cv2.imshow('r',im)
    #cv2.imshow('thres',edged)
    #cv2.waitKey(100)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cvImage, "bgr8"))
    except CvBridgeError, e:
      print e

def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
