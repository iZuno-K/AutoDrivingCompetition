## Original scenario
```bash
$ roslaunch aichallenge_bringup aichallenge_bringup.launch
```
### Scenario
```bash
$ roscd aichallenge_bringup/scenario/
$ python3 scenario.train.py 
```
## 交差点専用シナリオの追加
- 1番最初に左折するの交差点（saved_waypoints.csvの#382）
- 2番目に左折するの交差点（saved_waypoints.csvの#458）

### 例) 2番目交差点の場合
- roslaunch
```bash
$ roslaunch aichallenge_bringup intersection2.launch
```
- Scenario
```bash
$ roscd aichallenge_bringup/scenario/
$ python3 intersection2.py 
```

### 開始地点を自分で設定したい場合
- いじるならintersection2のペアをいじることがオススメ
- TFのイニシャライザ`initial_pose_publisher_intersection2.py`の改変
	- `saved_waypoints.csv`のx, z, y, yawをコピペ
	- `initial_pose_publisher_intersection2.py`のList`x_y_z_yaw = `にセット
- シミュレータ上のイニシャライザ`intersection2.py`の改変
	- `saved_waypoints.csv`のx, z, y, yawをコピペ
	- `intersection2.py`のList`x_y_z_yaw = `にセット


## Dependencies
### YOLO
- [url](https://github.com/Autoware-AI/core_perception/tree/master/vision_darknet_detect)
- Usage
```
cd aichallenge-bringup-final/data/
wget https://pjreddie.com/media/files/yolov3.weights
```

### 
- Usage
```
$ cd aichallenge-bringup-final/data/
$ wget https://github.com/k0suke-murakami/kitti_pretrained_point_pillars/raw/master/pfe.onnx
$ wget https://github.com/k0suke-murakami/kitti_pretrained_point_pillars/raw/master/rpn.onnx

```

### Others
- ndt_matching
  - Localization with LiDAR
  - readmeが見つからない

- [naive_motion_predict](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/tree/b711f8d30873288bab8190b4c2ed2b904c02a3c6/naive_motion_predict)
  - Object motion predictor
  - `imm_ukf_pda_track_lanelet2`から指令を受け取って、cost map generatorに値を渡す

- [imm_ukf_pda_track_lanelet2](https://gitlab.com/autowarefoundation/autoware.ai/core_perception/-/tree/57828ecf39ff62f8895f9eb66dbaadb5d610118e/lidar_imm_ukf_pda_track)
  - Object tracker with `tf` and fused sensor data

- [costmap_generator_lanelet2](https://gitlab.com/autowarefoundation/autoware.ai/core_planning/-/tree/master/costmap_generator)
  - Generate costmap 

## Done
- 

## To do 
- 