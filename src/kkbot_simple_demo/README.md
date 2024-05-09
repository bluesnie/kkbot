# 要想跑该demo记得把`aws-robomaker-small-warehouse-world`和其他相关的`models`拷贝到 `~/.gazebo/models/`下

# DEBUG
- 注释`nav_to_pose_example_launch.py`下的`ld.add_action(demo_cmd)`，编译
- 命令行启动其他节点`ros2 launch kkbot_simple_demo nav_to_pose_example_launch.py`
- `vscode`启动`debug`下的`Python: Current File`，就可以调试该节点了
> 为什么不直接启动`launch.py`，会有其它节点不启动，待查原因