# 现场调试指令速查

## 1. 基础环境

```bash
cd /SDCARD/uran_ws
source /opt/ros2/galactic/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
```

如果机器狗使用了命名空间，先看：

```bash
ros2 topic list | grep -E 'scan|odom_out|ubx|uran'
```

## 2. 切到自动模式

```bash
ros2 service call /uran/core/state/set uran_srvs/srv/SetStateField "{field_name: 'control_mode', value_json: '\"auto\"', persistent: false}"
ros2 service call /uran/core/state/set uran_srvs/srv/SetStateField "{field_name: 'current_controller', value_json: '\"auto\"', persistent: false}"
ros2 topic pub --times 5 --rate 2 /uran/core/switch/mode uran_msgs/msg/ModeSwitchCmd "{control_mode: auto, controller: auto, timestamp_ns: 0}"
```

确认：

```bash
ros2 service call /uran/autotask/status uran_srvs/srv/GetTaskStatus "{task_id: ''}"
```

## 3. GPS 检查

```bash
ros2 topic echo /ubx/status --field data --full-length
ros2 topic echo /ubx/gps_payload
ros2 topic hz /ubx/gps_payload
```

重点看：

```text
connected
stale
valid_fix
fix_type
num_sv
lat
lon
last_error
```

## 4. 雷达和里程计检查

```bash
ros2 topic list -t | grep -E 'scan|LaserScan|odom_out|Odometry'
ros2 topic hz /mi_desktop_48_b0_2d_5f_b6_d0/scan
ros2 topic hz /mi_desktop_48_b0_2d_5f_b6_d0/odom_out
```

## 5. 手动下发直线避障任务

```bash
ros2 topic pub --times 1 /uran/core/downlink/task_ctrl uran_msgs/msg/TaskCtrlCmd "{msg_version: '1.0', task_id: 'straight_test_001', action: 'start', task_type: 'straight_drive', task_params_json: '{\"task_type\":\"straight_drive\",\"distance_m\":2.0,\"speed_mps\":0.12}', timestamp_ns: 0}"
```

监控速度指令：

```bash
ros2 topic echo /uran/core/downlink/move_cmd
```

## 6. 手动打断任务

```bash
ros2 topic pub --times 1 /uran/core/downlink/task_ctrl uran_msgs/msg/TaskCtrlCmd "{msg_version: '1.0', task_id: '', action: 'stop', task_type: 'mission_planner_route', task_params_json: '{\"task_type\":\"mission_planner_route\"}', timestamp_ns: 0}"
```

## 7. 只读失败码

```bash
ros2 service call /uran/autotask/status uran_srvs/srv/GetTaskStatus "{task_id: ''}" | python3 -c "import sys,re,ast,json; s=sys.stdin.read(); m=re.search(r\"status_json='(.*)'\\)\\s*$\", s, re.S); d=json.loads(ast.literal_eval(\"'\"+m.group(1)+\"'\")); e=d.get('error') or {}; print('stage:',d.get('stage')); print('status:',d.get('status')); print('event:',d.get('event')); print('code:',e.get('code','')); print('description:',e.get('description','')); print('suggested_action:',e.get('suggested_action',''))"
```

## 8. 看方向校准状态

```bash
ros2 service call /uran/autotask/status uran_srvs/srv/GetTaskStatus "{task_id: ''}" | python3 -c "import sys,re,ast,json; s=sys.stdin.read(); m=re.search(r\"status_json='(.*)'\\)\\s*$\", s, re.S); d=json.loads(ast.literal_eval(\"'\"+m.group(1)+\"'\")); o=d.get('outdoor_navigation') or {}; print(json.dumps({'initial_yaw_calibration_active': o.get('initial_yaw_calibration_active'), 'local_nav_yaw_offset_deg': o.get('local_nav_yaw_offset_deg'), 'gps_vo_yaw_alignment': o.get('gps_vo_yaw_alignment')}, ensure_ascii=False, indent=2))"
```

## 9. 监控任务上行

```bash
ros2 topic echo /uran/core/uplink/data
ros2 topic hz /uran/core/uplink/data
```

## 10. 编译和重启

```bash
cd /SDCARD/uran_ws
colcon build --packages-select uran_autotask
```

编译后需要重启运行 `uran_autotask` 的服务或进程。

外接 GPS 服务：

```bash
cd /SDCARD/ubx_ws
colcon build --packages-select ubx_gps
systemctl --user restart ubx_gps.service
systemctl --user status ubx_gps.service --no-pager -l
```

如果工作区是复制过来的，先清缓存：

```bash
rm -rf build install log
colcon build --packages-select uran_autotask
```
