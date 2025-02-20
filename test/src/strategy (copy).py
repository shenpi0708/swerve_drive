#!/usr/bin/env python3
# license removed for brevity
import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.msg import *



xx = 0
yy = 0
zz = 0

pos = [0,0]
front = 0
data = Twist()
global_pub = rospy.Publisher('/vehicle/cmd_global', Twist, queue_size=1)
# def Pub(topic):
#     global data
#     topic.publish(data)

def Set_Odom(msg):
    global pos,front
    pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    # quaternion to euler
    (Alpha, Beta, front) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,
                                                                        msg.pose.pose.orientation.y,
                                                                        msg.pose.pose.orientation.z,
                                                                        msg.pose.pose.orientation.w])
    

    
    

def global_cmd(msg):
    global data
    data = msg
    # data.angular.z=data.angular.z+front
    # print(data)

def transform():
    global data  , front
    pub = Twist()
    pub.linear.x=data.linear.x*math.cos(front)+data.linear.y*math.sin(front)
    pub.linear.y=-data.linear.x*math.sin(front)+data.linear.y*math.cos(front)
    pub.angular.z=data.angular.z
    print(round(front*180/math.pi,3))
    # print(data.linear.x*math.cos(front),data.linear.y*math.sin(front))
    return pub

def main():
    global front,data
    rospy.Subscriber("/vehicle/odom_new", Odometry, Set_Odom)
    rospy.Subscriber("/vehicle/cmd_new", Twist, global_cmd)
    
    rospy.init_node('strategy', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        
        global_pub.publish(transform())
        # print(data)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
