#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import time
import math
waypoint = [(7, 0), (6, 0), (5, 0), (5, 1), (5, 2), (5, 3), (4, 3), (3, 3), (3, 2), (2, 2), (2, 1), (1, 1), (1, 0)]
i = 1
nob = 2
n = 0
# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = waypoint[0][0]*0.28 +  0.216829
desired_position_.y = waypoint[0][1]*0.28 -  2.443509 
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.05
exit_grid = False

# publishers
pub = None

def push_box():
    print("Utha le re deva")
    # desired_position_.x += 0.17
    desired_position_.y += 0.15

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)


def change_state(state):
    global state_
    state_ = state
    print 'State changed to [%s]' % state_


def clbk_odom(msg):
    global position_
    global yaw_
    
    # position
    position_ = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def waypoint_generator():
    global i, waypoint, yaw_
    try :
        if i == len(waypoint)-1:
            if n > 1:
                desired_position_.x = waypoint[i][0]*0.28 + 0.216829
                desired_position_.y = waypoint[i][1]*0.28 - 2.443509
                done()
            else:
                desired_position_.x = waypoint[i][0]*0.28 + 0.216829
                desired_position_.y = waypoint[i][1]*0.28 - 2.443509 + 0.17
                dist_precision_ = 0.025
        else:
            desired_position_.x = waypoint[i][0]*0.28 + 0.216829
            desired_position_.y = waypoint[i][1]*0.28 - 2.443509
    except Exception as e:
        print(e)

def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_, i, waypoint
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
    if err_pos > dist_precision_:
        twist_msg = Twist()
        # if i == len(waypoint)-1:
        #     twist_msg.linear.x = 0.2
        # else:
        twist_msg.linear.x = 0.15
        pub.publish(twist_msg)
    else:
        print 'Position error: [%s]' % err_pos
        i += 1
        if exit_grid :
            done()
        waypoint_generator()
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(0)


def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    if -math.pi > err_yaw :
        err_yaw = err_yaw + 2*math.pi
    if err_yaw > math.pi :
        err_yaw = err_yaw - 2*math.pi     
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 7 if err_yaw > 0 else -7
    
    pub.publish(twist_msg)
    print(err_yaw)

    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_state(1)




def main():
    global pub, n, nob, i, waypoint
    
    rospy.init_node('go_to_point')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if state_ == 0:
            print("hello")
            fix_yaw(desired_position_)
        elif state_ == 1:
            go_straight_ahead(desired_position_)
        elif state_ == 2:
            if i > len(waypoint)-1 :
                print("Kuch to gadbad hai daya")
                if n < nob :
                    # push_box()
                    # time.sleep(10)
                    i=0
                    if n == 0:
                        waypoint = [(1, 0), (1, 1), (2, 1), (2, 2), (3, 2), (3, 3), (3, 4), (4, 4), (5, 4), (6, 4), (7, 4), (8, 4)]

                    elif n == 1 :
                        waypoint = [(6, 8), (6, 7), (7, 7), (8, 7), (8, 8)]
                    n += 1
                else :
                    exit_grid = True
                    desired_position_.x = 8*0.28 + 0.216829 + 0.06
                    desired_position_.y = 8*0.28 - 2.4435091
                    go_straight_ahead(desired_position_)
                    print("aur kasa kai")
            else:
                change_state(0) 
        else:
            rospy.logerr('Unknown state!')
        rate.sleep()

if __name__ == '__main__':
    main()
# Junction 5
      # x: -0.994028589323
      # y: -1.18035435963
# Junction 14
      # x: -1.00867119334
      # y: -0.860871263664
    #   x: -2.114245
    #   y: -1.207871