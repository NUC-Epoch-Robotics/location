# location
使用fastlio2 slamtoolbox 

已知地图定位

```shell
ros2 launch robot_nav_bringup bringup_real.launch.py \
world:=YOUR_WORLD_NAME \
mode:=nav \
lio:=fastlio \
localization:=slam_toolbox \
lio_rviz:=False \
nav_rviz:=True
```

边建图边定位
```
ros2 launch robot_nav_bringup bringup_real.launch.py \
world:=YOUR_WORLD_NAME \
mode:=mapping  \
lio:=fastlio \
lio_rviz:=True

```
查看3d和2d地图
```
pcl_viewer basketball_ground.pcd 
eog basketball_ground.pgm