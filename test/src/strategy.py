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

pos = [0,0]
front = 0

# parameter
distance_error = 0.1
angle_error = 3

xy_reducer = 0.01
z_reducer = 0.7



def Set_Odom(msg):
    global pos,front
    pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    # quaternion to euler
    (Alpha, Beta, front) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,
                                                                        msg.pose.pose.orientation.y,
                                                                        msg.pose.pose.orientation.z,
                                                                        msg.pose.pose.orientation.w])
    front = front*180/math.pi
    # print(round(pos[0],3),round(pos[1],3),round(front*180/math.pi,3))

def Pub_cmd(x,y,z,topic):
    data = Twist()
    data.linear.x = x
    data.linear.y = y
    data.angular.z = z/180*math.pi
    topic.publish(data)
    
def XY_speed_planing(dis):
    global xy_reducer, distance_error,x_min_speed
    full_speed_dis = 0.3 # meter
    slow_speed_dis = 0.1 # meter

    if(abs(dis)>=full_speed_dis):
        return xy_reducer*dis/abs(dis)*100
    elif(full_speed_dis>abs(dis) and abs(dis)>=slow_speed_dis):
        return xy_reducer*100*dis/full_speed_dis
    elif(slow_speed_dis>abs(dis) and abs(dis)>=distance_error):
        return xy_reducer*dis/abs(dis)*20
    else:
        return 0 

def z_speed_planing(ang):
    global z_reducer, angle_error,z_min_speed
    full_speed_ang = 20
    slow_speed_ang = 5

    if(abs(ang)>=full_speed_ang):
        return z_reducer*abs(ang)/ang*100
    elif(full_speed_ang>abs(ang) and abs(ang)>=slow_speed_ang):
        return z_reducer*100*ang/full_speed_ang
    elif(slow_speed_ang>abs(ang) and abs(ang)>angle_error):
        return z_reducer*abs(ang)/ang*20
    else:
        return 0

def movement(t_x, t_y, t_z):
    global pos,front,angle_error
    x_speed = 0
    y_speed = 0
    z_speed = 0

    x_speed = XY_speed_planing(t_x-pos[0])

    y_speed = XY_speed_planing(t_y-pos[1])

    z_speed = z_speed_planing(t_z-front)

    # print(x_speed,y_speed,z_speed)
    return x_speed, y_speed, z_speed

def main():
    rospy.Subscriber("/vehicle/odom_new", Odometry, Set_Odom)
    move_pub = rospy.Publisher('/vehicle/cmd_new', Twist, queue_size=1)
    rospy.init_node('strategy', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    time = 0
    while not rospy.is_shutdown():
        if(time<=20):
            # print("1")
            xx,yy,zz = -0.5,-0.1,0
        else:
            # print("2")
            xx,yy,zz = -0.5,0.1,0
        
        time += 1
        if(time>40):
            time = 0

        # xx,yy,zz = movement(-0.0,0.0,0.0)
        # print(xx,yy,zz)
        Pub_cmd(xx,yy,zz,move_pub)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
