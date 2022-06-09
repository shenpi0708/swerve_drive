# swerve_drive
AMR using swerve drive wheel

update maxon library
https://www.maxongroup.com.tw/maxon/view/product/control/Positionierung/347717

download library
$ sudo apt install ros-noetic-velocity-controllers


lunch file
$ killall gzserver
$ killall gzclient
$ roslaunch vehicle_controller vehicle_controller.launch sim:=true

Xian launch
$ roslaunch vehicle_controller vehicle_test.launch

