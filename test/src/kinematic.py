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

robot_width = 0.36
robot_length = 0.535

xx = 0
yy = 0
zz = 0

pos = [0,0]
front = 0


def Rad2Deg(angle):  # radius to degree
    return angle * 180 / math.pi

def Set_Odom(msg):
    global pos,front
    pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    # quaternion to euler
    (Alpha, Beta, Gamma) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,
                                                                        msg.pose.pose.orientation.y,
                                                                        msg.pose.pose.orientation.z,
                                                                        msg.pose.pose.orientation.w])
    front = Rad2Deg(Gamma)
    # print(pos[0],pos[1],front)

def Sub_cmd_Vel(msg):
    global xx, yy, zz
    xx = msg.linear.x
    yy = msg.linear.y
    zz = msg.angular.z
    print(xx,yy,zz)

def Pub_lf_angle(topic, z):
    data = Float64()
    data.data = z
    topic.publish(data)

def Pub_lf_v(topic, x):
    data = Float64()
    data.data = -x
    topic.publish(data)

def Pub_rf_angle(topic, z):
    data = Float64()
    data.data = z
    topic.publish(data)

def Pub_rf_v(topic, x):
    data = Float64()
    data.data = x
    topic.publish(data)

def Pub_lr_angle(topic, z):
    data = Float64()
    data.data = z
    topic.publish(data)
    topic.publish(data)

def Pub_lr_v(topic, x):
    data = Float64()
    data.data = -x
    topic.publish(data)

def Pub_rr_angle(topic, z):
    data = Float64()
    data.data = z
    topic.publish(data)

def Pub_rr_v(topic, x):
    data = Float64()
    data.data = x
    topic.publish(data)

def Inverse_kinematic(Vx,Vy,Wp):
    # angle unit radian
    # print (Vx,Vy,Wp)
    L = robot_length
    W = robot_width
    alpha = math.atan(W/L)
    beta = math.atan(L/W)

    # Robot coordinates
    if(Vx==0 and Vy == 0 and Wp==0):
        A1 = A2 = A3 = A4 = 0
        V1 = V2 = V3 = V4 = 0
    elif(Vx==0 and Wp==0):
            A1 = A2 = A3 = A4 = math.pi/2*Vy/abs(Vy)
            V1 = V2 = V3 = V4 = abs(Vy)
    else:
        A1 = math.atan((Vy+L*Wp/2)/(Vx-W*Wp/2))
        A2 = math.atan((Vy-L*Wp/2)/(Vx-W*Wp/2))
        A3 = math.atan((Vy-L*Wp/2)/(Vx+W*Wp/2))
        A4 = math.atan((Vy+L*Wp/2)/(Vx+W*Wp/2))
        # V1 = Vx*math.cos(A1)+Vy*math.sin(A1)+Wp*math.sqrt(W*W+L*L)/2*math.sin(-A1)
        # V2 = Vx*math.cos(A2)+Vy*math.sin(A2)+Wp*math.sqrt(W*W+L*L)/2*math.sin(+A2)
        # V3 = Vx*math.cos(A3)+Vy*math.sin(A3)-Wp*math.sqrt(W*W+L*L)/2*math.sin(-A3)
        # V4 = Vx*math.cos(A4)+Vy*math.sin(A4)-Wp*math.sqrt(W*W+L*L)/2*math.sin(+A4)

        V1 = Vx*math.cos(A1)+Vy*math.sin(A1)+Wp*math.sqrt(W*W+L*L)/2*math.cos(1*math.pi/2+alpha-A1)
        V2 = Vx*math.cos(A2)+Vy*math.sin(A2)+Wp*math.sqrt(W*W+L*L)/2*math.cos(2*math.pi/2+beta-A2)
        V3 = Vx*math.cos(A3)+Vy*math.sin(A3)+Wp*math.sqrt(W*W+L*L)/2*math.cos(3*math.pi/2+alpha-A3)
        V4 = Vx*math.cos(A4)+Vy*math.sin(A4)+Wp*math.sqrt(W*W+L*L)/2*math.cos(0*math.pi/2+beta-A4)

        # theta = theta2 = 0
        # w = Wp
        # r = math.sqrt(W*W+L*L)/2
        # V1=math.sin(A1-theta)+math.cos(A1+theta)+w*r*math.sin(theta2-A1)
        # V2=math.sin(A2-theta)+math.cos(A2+theta)+w*r*math.sin(theta2+A2)
        # V3=math.sin(A3-theta)+math.cos(A3+theta)-w*r*math.sin(theta2-A3)
        # V4=math.sin(A4-theta)+math.cos(A4+theta)-w*r*math.sin(theta2+A4)

    return V1, A1, V2, A2, V3, A3, V4, A4

def main():
    lf_v_pub = rospy.Publisher('/lf_wheel/wheel_controller/command', Float64, queue_size=1)
    lf_angle_pub = rospy.Publisher('/lf_wheel/swerve_controller/command', Float64, queue_size=1)
    rf_v_pub = rospy.Publisher('/rf_wheel/wheel_controller/command', Float64, queue_size=1)
    rf_angle_pub = rospy.Publisher('/rf_wheel/swerve_controller/command', Float64, queue_size=1)
    lr_v_pub = rospy.Publisher('/lr_wheel/wheel_controller/command', Float64, queue_size=1)
    lr_angle_pub = rospy.Publisher('/lr_wheel/swerve_controller/command', Float64, queue_size=1)
    rr_v_pub = rospy.Publisher('/rr_wheel/wheel_controller/command', Float64, queue_size=1)
    rr_angle_pub = rospy.Publisher('/rr_wheel/swerve_controller/command', Float64, queue_size=1)
    rospy.Subscriber("/test_cmd_Vel", Twist, Sub_cmd_Vel)
    rospy.Subscriber("/vehicle/odom", Odometry, Set_Odom)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        V1, A1, V2, A2, V3, A3, V4, A4 = Inverse_kinematic(xx, yy, zz)
        Pub_lf_angle(lf_angle_pub, A1)
        Pub_lf_v(lf_v_pub, V1)
        Pub_lr_angle(lr_angle_pub, A2)
        Pub_lr_v(lr_v_pub, V2)
        Pub_rr_angle(rr_angle_pub, A3)
        Pub_rr_v(rr_v_pub, V3)
        Pub_rf_angle(rf_angle_pub, A4)
        Pub_rf_v(rf_v_pub, V4)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
