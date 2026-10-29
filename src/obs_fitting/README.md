启动雷达并进行数据转换：
1.source devel/setup.bash
2.roslaunch hesai_ros_driver start.launch 
3.新开窗口 source devel/setup.bash
4.roslaunch hesai_to_velodyne hesai_to_velodyne.launch 

障碍物拟合：
1.开启高性能模式
sudo nvpmodel -m 0
sudo jetson_clocks
2.source devel/setup.bash
3.roslaunch obs_fitting obs_fitting.launch 
