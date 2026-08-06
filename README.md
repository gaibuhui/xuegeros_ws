## Setup

Install [devenv](https://devenv.sh/getting-started/) before building the workspace.

Clone the micro-ROS dependencies, then apply the macOS Fast DDS compatibility
patch before building the workspace:

```sh
git submodule update --init --recursive
./scripts/apply-micro-ros-agent-patch.sh
devenv shell -- colcon build --packages-select micro_ros_msgs micro_ros_agent
```

1. 启动 agent 让机器连

    micro-ros-agent

2. 开启摄像头

    ros2 launch xuegecar_camera http_video_publisher.launch.py

3. 开启雷达

    ros2 launch xuegecar_bringup xuegecar_bringup.launch.py


4. 启动雷达时间戳转换

    ros2 launch scan_timestamp_relay scan_timestamp_relay.launch.py

5. 选择一个建图算法启动（只能启动其中一个，均订阅 /scan_synced）

    # GMapping
    ros2 launch slam_gmapping slam_gmapping.launch.py

    # Cartographer
    ros2 launch xuegecar_cartographer cartographer.launch.py

6. 虚拟化

    ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765

7. 保存地图

    ros2 run nav2_map_server map_saver_cli -f room

8. 导航

    ros2 launch xuegecar_navigation2 navigation2.launch.py
