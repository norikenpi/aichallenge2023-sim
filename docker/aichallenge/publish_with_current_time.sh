#!/bin/bash

# /clock トピックから時刻を取得
TIME_DATA=$(ros2 topic echo /clock --once | grep -E "sec|nanosec")

# sec と nanosec を抽出
SEC=$(echo "$TIME_DATA" | grep "^  sec:" | awk '{print $2}')
NANOSEC=$(echo "$TIME_DATA" | grep "nanosec:" | awk '{print $2}')

# 変数の値を出力
echo "TIME_DATA: $TIME_DATA"
echo "SEC: $SEC"
echo "NANOSEC: $NANOSEC"

# ros2 topic pub コマンドに時刻を挿入して実行
ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: $SEC, nanosec: $NANOSEC}, frame_id: 'map'}, pose: {position: {x: 3720.764404296875
, y: 73737.8671875
, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.9756383058075038
, w: 0.21938526897005559}}}" --once

# make vehivle automate
source ~/autoware/install/setup.bash
ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode {}
