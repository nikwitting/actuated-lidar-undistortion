# actuated-lidar-undistortion

Setup:
This repository is a fork of A-LOAM and the same prerequisites need to be installed as 1.1, 1.2 and 1.3 here: https://github.com/HKUST-Aerial-Robotics/A-LOAM


Then instead of cloning A-LOAM do the following:
```
cd ~/catkin_ws/src
git clone https://github.com/nikwitting/actuated-lidar-undistortion
cd ../
catkin_make -DCMAKE_BUILD_TYPE=Release
source ~/catkin_ws/devel/setup.bash
```


Download sample data with an actuated LiDAR:
https://drive.google.com/file/d/1XdTG3SuC--g45q5V_1-Ho7um4K9zmOFy/view?usp=sharing

To run the software:

Open a terminal
```
source ~/catkin_ws/devel/setup.bash
roslaunch aloam_velodyne eth_spin.launch
```

Open another terminal
```
rosbag play second.bag --topic /velodyne_points /imu0 --clock
```
NOTE: allways use --clock
