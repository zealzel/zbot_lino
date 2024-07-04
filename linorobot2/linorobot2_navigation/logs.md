### 2024/7/4 Fixes

#### In linorobot2_description/urdf/controllers/diff_drive.urdf.xacro, Add

```xml
<ros>
    <argument>odom:=odom/unfiltered</argument>
    <argument>/tf:=tf</argument>
  </ros>
```

#### In linorobot2_base/config/ekf.yaml

Change first line

`ekf_filter_node:`

to

`/**:`

#### Create linorobot2_navigation/config/zbotlino2/navigation.multiworked.yaml

difference with navigation.yaml

- amcl:
  [Add]

  map_topic: /map

- local_costmap.local_costmap
  [Add]

  map_topic: /map

- global_costmap.global_costmap
  [Add]

  map_topic: /map

- velocity_smoother:
  [Mod]

  odom_topic: odom -> /odom

#### Use modified nav2_bringup locally

#### In linorobot2/linorobot2_description/urdf/sensors/ultrasonic.urdf.xacro

ROS2 中沒有 libgazebo_ros_range.so, 合併到 libgazebo_ros_ray_sensor.so

Change

`<plugin filename="libgazebo_ros_range.so" name="gazebo_ros_range">`

Into

`<plugin filename="libgazebo_ros_ray_sensor.so" name="${frame_id}_gazebo_ros_range">`
