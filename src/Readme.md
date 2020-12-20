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