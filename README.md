# AutoDrivingCompetition

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
