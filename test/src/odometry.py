#!/usr/bin/env python3
# license removed for brevity
import rospy
import math
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import *
from geometry_msgs.msg import *

robot_width = 0.36
robot_length = 0.535

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

xx = 0.0
yy = 0.0
zz = 0.0

odom_broadcaster = tf.TransformBroadcaster()
current_time = rospy.Time.now()
last_time = rospy.Time.now()


def Pub_odom(vx, vy, vz, topic):
    global xx, yy, zz
    current_time = rospy.Time.now()

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * math.cos(vz) - vy * math.sin(vz)) * dt
    delta_y = (vx * math.sin(vz) + vy * math.cos(vz)) * dt
    delta_th = vz * dt

    xx += delta_x
    yy += delta_y
    zz += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, zz)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (xx, yy, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vz))

    # publish the message
    topic.publish(odom)

    last_time = current_time


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

def Joint_to_odom(lf_a,lf_v,lr_a,lr_v,rf_a,rf_v,rr_a,rr_v):

    L = robot_length
    W = robot_width
    alpha = math.atan(W/L)
    beta = math.atan(L/W)

    xx = math.cos(lf_a)*lf_v + math.cos(lr_a)*lr_v + math.cos(rr_a)*rr_v + math.cos(rf_a)*rf_v
    yy = math.sin(lf_a)*lf_v + math.sin(lr_a)*lr_v + math.sin(rr_a)*rr_v + math.sin(rf_a)*rf_v
    zz = math.sqrt(W*W+L*L)/2*math.cos(1*math.pi/2+alpha-lf_a)*lf_v + math.sqrt(W*W+L*L)/2*math.cos(2*math.pi/2+beta-lr_a)*lr_a + math.sqrt(W*W+L*L)/2*math.cos(3*math.pi/2+alpha-rr_a)*rr_v + math.sqrt(W*W+L*L)/2*math.cos(0*math.pi/2+beta-rf_a)*rf_v
    print(xx, yy, zz)
    return xx,yy,zz

def main():

    rospy.Subscriber("/lf_wheel/joint_states", JointState, Sub_lf_joint)
    rospy.Subscriber("/rf_wheel/joint_states", JointState, Sub_rf_joint)
    rospy.Subscriber("/lr_wheel/joint_states", JointState, Sub_lr_joint)
    rospy.Subscriber("/rr_wheel/joint_states", JointState, Sub_rr_joint)
    odom_pub = rospy.Publisher('/vehicle/odom_new', Odometry, queue_size=1)
    rospy.init_node('odometry', anonymous=True)
    rate = rospy.Rate(20) # 10hz

    while not rospy.is_shutdown():
        Vx,Vy,Vz = Joint_to_odom(lf_angle,lf_velocity,lr_angle,lr_velocity,rf_angle,rf_velocity,rr_angle,rr_velocity)
        Pub_odom(Vx, Vy, Vz, odom_pub)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
