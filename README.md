## Setup

Install [devenv](https://devenv.sh/getting-started/) before building the workspace.

Clone the micro-ROS dependencies, then apply the macOS Fast DDS compatibility
patch before building the workspace:

```sh
git submodule update --init --recursive
./scripts/apply-micro-ros-agent-patch.sh
devenv shell -- colcon build --packages-select micro_ros_msgs micro_ros_agent
```

## 基础服务

启动 micro-ROS Agent、摄像头、底层驱动、`/scan_synced` 时间戳中继和
Foxglove：

```sh
devenv up
```

Foxglove Desktop 使用 `Foxglove WebSocket` 数据源连接 `ws://localhost:8765`。

`devenv up` 不启动建图、导航或避障，以避免建图算法选择冲突和 `/cmd_vel` 多发布者。

## 按需服务

1. 选择一个建图算法启动（只能启动其中一个，均订阅 `/scan_synced`）

    ```sh
    # GMapping
    ros2 launch slam_gmapping slam_gmapping.launch.py

    # Cartographer
    ros2 launch xuegecar_cartographer cartographer.launch.py
    ```

2. 保存地图

    ros2 run nav2_map_server map_saver_cli -f room

3. 导航

    ros2 launch xuegecar_navigation2 navigation2.launch.py

## 雷达自主避障

避障节点独占 `/cmd_vel`。启动前关闭 Nav2、手柄和其他速度发布者，并先确认
micro-ROS Agent、雷达与唯一的 `/scan_synced` 时间戳中继已经运行：

```sh
devenv shell -- colcon build --packages-select xuegecar_obstacle_avoidance
source install/setup.bash
ros2 launch xuegecar_obstacle_avoidance obstacle_avoidance.launch.py
```

节点默认以低速前进；前方障碍物距离小于 `0.35` 米时停止直行并原地转向至净空更大的
一侧。转向 `2` 秒仍找不到可前进方向时会尝试另一侧；两侧均无结果且后方至少有 `0.50`
米净空时，会以低速倒车 `1` 秒后继续搜索。只有雷达确认前、后、左、右都被近距离障碍
物封住时才停止；雷达数据超过 `0.5` 秒未更新时仍会安全停车，按 `Ctrl-C` 退出也会发送
零速度指令。倒车需要雷达覆盖车后方；无后方测距时节点会持续轮换转向搜索而不会盲目后退。
另外，节点使用 `/odom` 检测车底卡住或车轮打滑：持续发出前进或倒车指令 `2` 秒但位置移动
不到 `0.02` 米时，会立即触发上述恢复流程。即使轮编码器让 `/odom` 继续变化，节点也会
比较 4 米内至少 30 条有效雷达束；场景中位数变化小于 `0.04` 米同样判为打滑并脱困。可通过
launch 参数调整，例如：

节点还订阅 `/camera/image_raw/compressed` 和 `/imu`：连续平移中相机至少 20 个特征点的中位
运动小于 3 像素时会判为实际未移动；连续转向中 IMU 偏航变化小于 8 度也会进入倒车/换向恢复。

```sh
ros2 launch xuegecar_obstacle_avoidance obstacle_avoidance.launch.py \
  forward_speed:=0.08 stop_distance:=0.40 clear_distance:=0.55
```
