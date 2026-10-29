# 实车移植任务说明（给 AI 的 prompt）

> 用法：把本文件内容整体复制给工控机上的 AI 会话即可。

---

# 实车移植任务说明

## 我的需求
把我的一套阿克曼无人车 2D 导航栈从 x86 仿真机迁移到实车工控机（ARM/aarch64），在实车上跑通。仿真已经调好，算法代码都是调通可用的，不要改动算法本身，直接原样编译运行。

## 基本情况
- 两端都是 Ubuntu 20.04 + ROS Noetic；仿真机 x86_64，工控机 aarch64（Jetson 类）
- 实车上已部署运行着 LIO-SAM（里程计）和 Patchwork++（地面分割），直接对接复用
- 需要迁移的只有导航栈（在仿真工作区 src/ 下）：
  - src/ego-planner/src/planner/ 下 5 个包：plan_env、path_searching、bspline_opt、traj_utils、plan_manage
  - src/ego-planner/src/uav_simulator/Utils/waypoint_generator
  - 顶层 src/CMakeLists.txt
- 其他目录（lio_sam、patchworkpp、ackermann_vehicle_base_gazebo、hector_models、uav_simulator 其余部分）是仿真用的，不迁移
- 工作区用 catkin_tools（catkin build）
在/workspace/ros1_vehicles/src/obs_void/bringup/launch这个实车目录下的circular_path_navigation_gps_real.launch有我之前的避障算法的程序的启动流程，里面有一些你可以参考的，可能会用到的程序的位置，有用于做阿克曼解算的，/workspace/ros1_vehicles/src/obs_fitting这个目录下有patchwork++，/workspace/ros1_vehicles/src/localization/src/hesai_loam/LIO-SAM/launch有liosam，这两个直接通过launch启动就好，不用做任何修改
-所有的修改都不要更改我已经跑通的liosam和patchwork++，注意还有哪些参数是要因为迁移到实车做修改的

## 实车需要你确认/处理的点
- 底盘接口：直接收 /cmd_vel (Twist) 还是 /ackermann_cmd (ackermann_msgs)？后者需要 cmd_vel→ackermann 转换，
- launch 文件（plan_manage/launch/slam_navigation.launch）里有仿真残留 use_sim_time=true，实车必须去掉；lio-sam/patchwork 节点已在实车运行，不要重复启动，只核对话题对接
- 话题：LIO-SAM 输出 /lio_sam/mapping/odometry，Patchwork++ 输出 /patchworkpp/nonground，请按实车实际话题核对
- 仿真中车辆参数：wheelbase=0.335m，转向 ±45°，min_turning_radius=0.4，实车轴距0.66m,要定位到后轴中心点才能做ackerman解算，你需要了解现在的定位中心与后轴中心的位置关系
- 速度上限实车首跑建议降到 0.3~0.5 m/s（规划与 TEB 限速两处要同步）


## 期望你做的
1. 看懂现有代码结构和启动方式（5 包 + waypoint_generator，入口是 slam_navigation.launch）
2. 在工控机上完成依赖安装与编译
3. 按上述实车要点改配置并启动
4. 陪我一起在实车上验证：定位正常、前向跟踪、避障、到达终点停车
5. 与上述说明不符的实际情况以实车为准，并明确告诉我
