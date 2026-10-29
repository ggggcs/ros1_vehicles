#ros1_vehicles

设备上代码在~/workspace/ros1_vehicles
脚本在项目scripts


1. 编译
$ 
$ catkin_make


2. 启动
  分系统环境配置和程序启动


2.1 系统环境配置（开机后只启动一次）
$ sudo bash scripts/start_sys.sh


2.2 程序启动
$ bash scripts/start_sensor.sh start|stop #启动或停止所有传感器
$ bash scripts/calibration.sh # 启动标定节点


3. 录制
$ bash scripts/record_bag.sh（Ctrl-C停止录制）


4. 外出调试

见"docs/1_外出调试.txt"
