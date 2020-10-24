# lane_stopper

## Nodes
### lane_stopper

### braking_manager
- 単体評価
  - 1. `output="screen"` のバージョンを[roslaunch](launch/lane_stopper.launch)で使用
  - 2. [lane_stopper.cpp](nodes/lane_stopper.cpp)の`bool_publisher`の関与する箇所（3行）をコメントアウト
  - 3. Type following command, and messages `TO MA RE` is shown in `aichallenge-bringup.launch` launched terminal
  - 4. if `data: false`, `TO MA RE` will disappear
```bash
$ rostopic pub /brake_flag std_msgs/Bool "data: true"
```