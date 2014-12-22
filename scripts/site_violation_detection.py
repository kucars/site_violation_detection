#!/usr/bin/env python
import roslib
roslib.load_manifest('site_violation_detection')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    rospy.init_node('site_violation_detection', anonymous=True)
    self.image_pub = rospy.Publisher("/site_violation_detection/image_raw",Image, queue_size=10)
    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/image_raw",Image,self.callback)
    
  def callback(self,data):
    try:
      cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    (rows,cols,channels) = cvImage.shape

    cv2.circle(cvImage, (cols/2,rows/2), 225, (255,0,0),2)

    cv2.putText(cvImage,"Robotics Institute", (50, rows / 2), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255),2)
    cv2.putText(cvImage,"Khalifa University", (50, rows / 2 + 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,255,0),2)
    
    cv2.imshow("Image window", cvImage)
    cv2.waitKey(3)

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
