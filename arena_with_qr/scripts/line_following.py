#!/usr/bin/env python2

import rospy, cv2, numpy
from numpy import *
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations

class Follower():
  def __init__(self):
    rospy.init_node('follower')
    self.bridge = CvBridge()
    image = empty([]) 
    #cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('/rrbot/camera1/image_raw',Image, self.image_callback)
    self.sub_odom = rospy.Subscriber('/odom', Odometry, self.clbk_odom)
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)
    self.twist = Twist()
    self.position = Point()
    self.spawn_point = Point()
    self.spawn_point.x = -1.07
    self.spawn_point.y = -1.58
    self.spawn_point.z = 0.170097

  def done(self):
      self.twist.linear.x = 0
      self.twist.angular.z = 0
      self.cmd_vel_pub.publish(self.twist)

  def clbk_odom(self,msg):
      
      # position
      self.position = msg.pose.pose.position



  def image_callback(self, data):
    if abs(self.position.x - self.spawn_point.x) < 0.05 and abs(self.position.y - self.spawn_point.y) < 0.05 and abs(self.position.z - self.spawn_point.z) < 0.05 :
      self.done()
    image = self.bridge.imgmsg_to_cv2(data,"bgr8")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
    # change below lines to map the color you wanted robot to follow
    lower = numpy.array([ 0,  0,  140])
    upper = numpy.array([256, 60, 256])
    mask = cv2.inRange(hsv, lower, upper)
    
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      #cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      # CONTROL starts
      err = cx - w/2
      print(err)
      self.twist.linear.x = 0.1
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      # CONTROL ends
    cv2.imshow("mask",mask)
    cv2.imshow("output", image)
    cv2.waitKey(3)

if __name__ == '__main__':
  follower = Follower()
  rospy.spin()

