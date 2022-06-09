#!/usr/bin/env python3
# license removed for brevity
import rospy
import math
from std_msgs.msg import Float64
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.msg import *
from sensor_msgs.msg import *



robot_width = 0.36
robot_length = 0.535

xx = 0
yy = 0
zz = 0

pos = [0,0]
front = 0

lf_angle = 0.0
lf_velocity = 0.0
lr_angle = 0.0
lr_velocity = 0.0
rf_angle = 0.0
rf_velocity = 0.0
rr_angle = 0.0
rr_velocity = 0.0


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



def Sub_lf_joint(msg):
    global lf_angle, lf_velocity
    lf_angle = msg.position[0]
    lf_velocity = -msg.velocity[1]
    print("lf",lf_angle*180/math.pi,lf_velocity)

def Sub_rf_joint(msg):
    global rf_angle, rf_velocity
    rf_angle = msg.position[0]
    rf_velocity = msg.velocity[1]
    print("rf",rf_angle*180/math.pi,rf_velocity)

def Sub_lr_joint(msg):
    global lr_angle, lr_velocity
    lr_angle = msg.position[0]
    lr_velocity = -msg.velocity[1]
    print("lr",lr_angle*180/math.pi,lr_velocity)

def Sub_rr_joint(msg):
    global rr_angle, rr_velocity
    rr_angle = msg.position[0]
    rr_velocity = msg.velocity[1]
    print("rr",rr_angle*180/math.pi,rr_velocity)



def main():

    rospy.Subscriber("/lf_wheel/joint_states", JointState, Sub_lf_joint)
    rospy.Subscriber("/rf_wheel/joint_states", JointState, Sub_rf_joint)
    rospy.Subscriber("/lr_wheel/joint_states", JointState, Sub_lr_joint)
    rospy.Subscriber("/rr_wheel/joint_states", JointState, Sub_rr_joint)
    




    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        # V1, A1, V2, A2, V3, A3, V4, A4 = Inverse_kinematic(xx, yy, zz)
        # Pub_lf_angle(lf_angle_pub, A1)
        # Pub_lf_v(lf_v_pub, V1)
        # Pub_lr_angle(lr_angle_pub, A2)
        # Pub_lr_v(lr_v_pub, V2)
        # Pub_rr_angle(rr_angle_pub, A3)
        # Pub_rr_v(rr_v_pub, V3)
        # Pub_rf_angle(rf_angle_pub, A4)
        # Pub_rf_v(rf_v_pub, V4)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
