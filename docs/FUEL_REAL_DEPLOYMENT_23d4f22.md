# FUEL 安全桥真机部署与静态验证

目标版本：`23d4f2206e59eec1e9206748f86aacc1b7447c21`

部署包：

- 增量包：`/root/catkin_ws/backups/FUEL_143ec0f_to_23d4f22.bundle`
- 完整包：`/root/catkin_ws/backups/FUEL_safe_real_23d4f22.bundle`
- 增量包 SHA256：`2405f6889bfea5d76bfbd93973e3aa5d078e241f35389ba6b1b97327c52024a1`
- 完整包 SHA256：`6179f10796327ea3b8c28df4179ed13a81096b2e19aa66647fa225ba02d95ead`

## 安全前提

1. 拆除全部桨叶。
2. 飞控保持 `armed: false`。
3. 不发布 `/move_base_simple/goal`，不触发探索。
4. 不运行原来的 `run_single_lio.sh`，因为它会同时启动 diff-planner、px4ctrl 和 multipoint。
5. 发现无人机 FUEL 工作区有未提交修改时停止，不要覆盖或清除修改。

## 传输

在开发电脑执行：

```bash
scp /root/catkin_ws/backups/FUEL_143ec0f_to_23d4f22.bundle \
  nv@172.20.10.12:/home/nv/
```

在无人机电脑执行：

```bash
sha256sum /home/nv/FUEL_143ec0f_to_23d4f22.bundle
git -C /home/nv/fuel_flight_ws/src/FUEL status --short
git -C /home/nv/fuel_flight_ws/src/FUEL rev-parse HEAD
```

预期旧版本为 `143ec0f4dc414a41398d6c7ea83c84051d81939a`，并且工作区没有输出。符合条件后执行：

```bash
git -C /home/nv/fuel_flight_ws/src/FUEL fetch \
  /home/nv/FUEL_143ec0f_to_23d4f22.bundle main
git -C /home/nv/fuel_flight_ws/src/FUEL switch \
  -c fuel-safe-real-23d4f22 FETCH_HEAD
git -C /home/nv/fuel_flight_ws/src/FUEL rev-parse HEAD
```

如果无人机不在 `143ec0f`，改用完整 bundle；仍应先保存其当前状态，不要强制切换。

## 编译

使用 bash 终端：

```bash
source /opt/ros/noetic/setup.bash
source /home/nv/Diff-planner/devel/setup.bash
cd /home/nv/fuel_flight_ws
catkin_make -j4 -l4 --pkg fuel_command_safety
source /home/nv/fuel_flight_ws/devel/setup.bash
rospack find fuel_command_safety
rosmsg md5 quadrotor_msgs/PositionCommand
roslaunch --nodes fuel_command_safety fuel_safe_real_lio.launch
```

预期：

- `rospack` 指向 `/home/nv/fuel_flight_ws/src/FUEL/fuel_planner/fuel_command_safety`
- 消息 MD5 为 `2809eb0c779bbce5b8d66b95a05bd27b`
- launch 节点为 `/cloud_pose_adapter`、`/exploration_node`、`/traj_server`、`/waypoint_generator`、`/fuel_command_safety`

## 拆桨静态启动

每个终端都执行相同环境初始化：

```bash
source /opt/ros/noetic/setup.bash
source /home/nv/Diff-planner/devel/setup.bash
source /home/nv/fuel_flight_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
unset ROS_IP
unset ROS_HOSTNAME
```

依次在独立终端启动：

```bash
roscore
```

```bash
roslaunch mavros px4.launch
```

```bash
roslaunch faster_lio mapping_mid360.launch
```

```bash
roslaunch ekf ekf_lidar.launch
```

```bash
roslaunch px4ctrl run_ctrl_lio.launch
```

最后启动 FUEL 与安全桥，但不要发送探索目标：

```bash
roslaunch fuel_command_safety fuel_safe_real_lio.launch
```

## 静态验收

```bash
rostopic echo -n 1 /mavros/state
rostopic hz /ekf/ekf_odom
rostopic delay /ekf/ekf_odom
rostopic hz /laserMapping/cloud_registered
rostopic delay /laserMapping/cloud_registered
rostopic echo -n 1 /fuel_command_safety/state
rostopic info /fuel/position_cmd_raw
rostopic info /setpoints_cmd
rosnode list
```

必须同时满足：

1. `/mavros/state` 为 `connected: true`、`armed: false`。
2. `/fuel_command_safety/state` 为 `WAITING_INPUT`。
3. `/fuel/position_cmd_raw` 的发布者只有 `/traj_server`，订阅者包含 `/fuel_command_safety`。
4. `/setpoints_cmd` 的发布者只有 `/fuel_command_safety`，订阅者包含 px4ctrl。
5. 节点列表中没有 diff-planner 规划节点或 multipoint。
6. 尚未触发探索时，安全桥不应向 px4ctrl 发布运动轨迹。

完成后停止 FUEL 与安全桥。遥控器恢复并验证急停、模式开关和人工接管之前，不进入解锁或实飞步骤。
