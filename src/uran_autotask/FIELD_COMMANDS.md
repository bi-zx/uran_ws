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
ros2 topic pub --times 1 --rate 2 /uran/core/switch/mode uran_msgs/msg/ModeSwitchCmd "{control_mode: auto, controller: auto, timestamp_ns: 0}"
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

## 4. D430I 避障和里程计检查

```bash
systemctl --user status uran_realsense_navigation.service --no-pager -l
ros2 lifecycle get /camera/camera
ros2 topic hz /camera/depth/color/points
ros2 topic hz /uran/autotask/depth_scan
ros2 topic hz /mi_desktop_48_b0_2d_5f_b6_d0/odom_out
```

检查点云到机身坐标系的变换：

```bash
FRAME=$(timeout 3s ros2 topic echo /camera/depth/color/points --field header.frame_id | head -1 | tr -d " '\r")
echo "pointcloud frame: $FRAME"
ros2 run tf2_ros tf2_echo base_link "$FRAME"
```

只看 URAN 当前使用的障碍传感器状态：

```bash
ros2 service call /uran/autotask/status uran_srvs/srv/GetTaskStatus "{task_id: ''}" | python3 -c "import sys,re,ast,json; s=sys.stdin.read(); m=re.search(r\"status_json='(.*)'\\)\\s*$\", s, re.S); d=json.loads(ast.literal_eval(\"'\"+m.group(1)+\"'\")); print(json.dumps(d.get('obstacle_sensor') or {}, ensure_ascii=False, indent=2))"
```

正常时应看到 `source=depth_pointcloud`、`status=healthy`、
`converted_frames` 持续增加且 `last_error` 为空。`tf_unavailable` 表示相机到
`base_link` 的坐标变换缺失，URAN 会停车。

## 5. 手动下发直线避障任务

```bash
ros2 topic pub --times 1 /uran/core/downlink/task_ctrl uran_msgs/msg/TaskCtrlCmd "{msg_version: '1.0', task_id: 'straight_test_001', action: 'start', task_type: 'straight_drive', task_params_json: '{\"task_type\":\"straight_drive\",\"distance_m\":2.0,\"speed_mps\":0.8}', timestamp_ns: 0}"
```

监控速度指令：

```bash
ros2 topic echo /uran/core/downlink/move_cmd
```

监控 `uran_move` 实际执行、限速和拒绝结果：

```bash
ros2 topic echo /uran/core/uplink/data | grep -E 'move_result|move_reject_event|move_clamp_event|command_seq|actual_velocity|reason'
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
colcon build --packages-select uran_autotask uran_media uran_move

cd /SDCARD/uran_ws/scripts
./install_user_services.sh

systemctl --user restart uran_realsense_navigation.service
systemctl --user restart uran_media.service
systemctl --user restart uran_autotask.service
```

编译后需要重启 `uran_realsense_navigation`、`uran_media`、`uran_autotask` 和
`uran_move` 的服务或进程。

外接 GPS 服务：

```bash
cd /SDCARD/ubx_ws
colcon build --packages-select ubx_gps
systemctl --user restart ubx_gps.service
systemctl --user status ubx_gps.service --no-pager -l
```

如果工作区是复制过来的，先清缓存：

```bash
rm -rf build/uran_autotask build/uran_move install/uran_autotask install/uran_move
colcon build --packages-select uran_autotask uran_media uran_move
```

ubx_gps 是用户级服务，用这个：

```bash
cd /SDCARD/ubx_ws
colcon build --packages-select ubx_gps

systemctl --user daemon-reload
systemctl --user restart ubx_gps.service
systemctl --user status ubx_gps.service --no-pager -l
```

uran_autotask 如果是跟 cyberdog_bringup.service 一起启动的，用这个：

```bash
cd /SDCARD/uran_ws
colcon build --packages-select uran_autotask uran_move

systemctl --user restart uran_realsense_navigation.service
sudo systemctl restart cyberdog_bringup.service
sudo systemctl status cyberdog_bringup.service --no-pager -l
```

重启后检查：

```bash
export ROS_DOMAIN_ID=42
source /opt/ros2/galactic/setup.bash
source /SDCARD/uran_ws/install/setup.bash

ros2 topic echo /ubx/status --field data --full-length
ros2 service call /uran/autotask/status uran_srvs/srv/GetTaskStatus "{task_id: ''}"
```
