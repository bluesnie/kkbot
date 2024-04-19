
# 下载编译

- `git clone https://github.com/bluesnie/kkbot.git`
- `cd kkbot`
- `rosdep install --from-paths src -y`
- `colcon build`

#  运行

## RVIZ 显示模型

- `source install/setup.bash`
- `ros2 launch kkbot_description display_rviz2.launch.py`

## 仿真

- `source install/setup.bash`
- `ros2 launch kkbot_description gazebo.launch.py`

## 建图

- 启动仿真
- `source install/setup.bash`
- `ros2 launch kkbot_description gazebo.launch.py`
- 运行遥控：`ros2 run teleop_twist_keyboard teleop_twist_keyboard`

## nav2

- 启动仿真
- `source install/setup.bash`
- `ros2 launch kkbot_navigation2 navigation2.launch.py use_sim_time:=True`