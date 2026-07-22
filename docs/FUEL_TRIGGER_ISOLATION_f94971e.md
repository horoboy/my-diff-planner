# FUEL 真机触发入口隔离升级

目标版本：`f94971e0d8a9ee1d78651df5dfbb50f34e188ae3`

基线版本：`23d4f2206e59eec1e9206748f86aacc1b7447c21`

增量包：`/root/catkin_ws/backups/FUEL_23d4f22_to_f94971e.bundle`

SHA256：`97b2b94166c26e31abb6dea1e9f3f8a9425fdc57cd3a4dc8396c815e46939384`

## 变更原因

MAVROS 的 `guided_target` 插件会将飞控发来的全局位置目标发布到
`/move_base_simple/goal`。FUEL 真机 launch 原本也使用该话题作为人工探索入口，
存在非 FUEL 操作间接触发探索的风险。

升级后：

- 人工隔离入口：`/fuel/exploration_goal` -> `/waypoint_generator`
- 正式控制入口：`/traj_start_trigger` -> `/waypoint_generator`
- 规划触发：`/waypoint_generator/waypoints` -> `/exploration_node`
- `/move_base_simple/goal` 不再连接 FUEL

## 无人机升级

先停止 FUEL launch，保持飞控 `armed: false`，不要发送任何探索或起飞命令。
MAVROS、Faster-LIO、EKF 和 px4ctrl 可以继续运行。

开发电脑执行：

```bash
scp /root/catkin_ws/backups/FUEL_23d4f22_to_f94971e.bundle \
  nv@172.20.10.12:/home/nv/
```

无人机执行：

```bash
sha256sum /home/nv/FUEL_23d4f22_to_f94971e.bundle
git -C /home/nv/fuel_flight_ws/src/FUEL status --short
git -C /home/nv/fuel_flight_ws/src/FUEL rev-parse HEAD
```

工作区必须干净，HEAD 必须为 `23d4f22`。然后执行：

```bash
git -C /home/nv/fuel_flight_ws/src/FUEL fetch \
  /home/nv/FUEL_23d4f22_to_f94971e.bundle main
git -C /home/nv/fuel_flight_ws/src/FUEL switch \
  -c fuel-safe-trigger-f94971e FETCH_HEAD
git -C /home/nv/fuel_flight_ws/src/FUEL rev-parse HEAD
git -C /home/nv/fuel_flight_ws/src/FUEL status --short
```

本次只修改 launch XML，不需要重新编译 C++。静态检查：

```bash
source /opt/ros/noetic/setup.bash
source /home/nv/Diff-planner/devel/setup.bash
source /home/nv/fuel_flight_ws/devel/setup.bash
roslaunch --nodes fuel_command_safety fuel_safe_real_lio.launch
```

## 拆桨复验

只启动一次：

```bash
roslaunch fuel_command_safety fuel_safe_real_lio.launch
```

在另一个已加载相同环境的终端执行：

```bash
pgrep -af '[r]oslaunch.*fuel_safe_real_lio'
rosnode list | sort
rostopic info /move_base_simple/goal
rostopic info /fuel/exploration_goal
rostopic info /traj_start_trigger
rostopic info /waypoint_generator/waypoints
rostopic echo -n 1 /fuel_command_safety/state
rostopic echo -n 1 /mavros/state
```

必须满足：

1. 只有一个 `fuel_safe_real_lio` roslaunch 进程。
2. `/waypoint_generator` 存在。
3. `/move_base_simple/goal` 的订阅者中没有 `/waypoint_generator`。
4. `/fuel/exploration_goal` 的订阅者只有 `/waypoint_generator`，静态阶段没有发布者。
5. `/traj_start_trigger` 由 `/px4ctrl` 发布并由 `/waypoint_generator` 订阅。
6. `/waypoint_generator/waypoints` 连接到 `/exploration_node`。
7. 安全桥为 `WAITING_INPUT:no_valid_command_yet`，飞控为 `armed: false`。

遥控器、急停和人工接管恢复并验证前，不发布 `/fuel/exploration_goal`，不进入解锁或实飞。
