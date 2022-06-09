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

def Set_Odom(msg):
    global pos,front
    pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    # quaternion to euler
    (Alpha, Beta, front) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,
                                                                        msg.pose.pose.orientation.y,
                                                                        msg.pose.pose.orientation.z,
                                                                        msg.pose.pose.orientation.w])

    print(pos[0],pos[1],front*180 / math.pi)



def main():
    rospy.Subscriber("/vehicle/odom_new", Odometry, Set_Odom)
    rospy.init_node('strategy', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
