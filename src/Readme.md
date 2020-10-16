## YOLO
- [Package](https://github.com/Autoware-AI/core_perception/tree/master/vision_darknet_detect)
```
cd aichallenge-bringup-final/data/
wget https://pjreddie.com/media/files/yolov3.weights
```


## Done
- 

## To do 
- `naive motion predict`の動作確認
- (頻度中)人間回避後の挙動が不安定になる
  - Followings are example;
    - `[ERROR] [1602858751.154365983]: Can't find goal...`
    - `[ INFO] [1602858751.154472857]: PLANNING -> STOPPING, Cannot find path`
    - `[ WARN] [1602860181.893785937]: Current_pose is far away from previous closest waypoint. Initilized...`
  - waypointによる制御に回避計画をどうマージさせるかを検討するべきかも
