# swerve_drive
AMR using swerve drive wheel

update maxon library
https://www.maxongroup.com.tw/maxon/view/product/control/Positionierung/347717

download library
$ sudo apt install ros-noetic-velocity-controllers


lunch files
    $ killall gzserver
    $ killall gzclient
    $ roslaunch vehicle_controller vehicle_controller.launch sim:=true
    or
    $ roslaunch vehicle_controller vehicle_test.launch sim:=true

run files
    $ rosrun test kinematic.py
    $ rosrun test odometry.py

need to be improved
    1. The direction of wheeld velocity on the left side is opposite in real world robot
    2. /vehicle/cmd is a little be weird, /test_cmd_Vel is ok, but it's a python file
    3. /vehicle/odom is weird
    4. motor feedback is not working in gazebo


