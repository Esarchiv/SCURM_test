# Navigation2 记录

## 1.建图 (已source) png格式的栅格地图

```
ros2 launch pb_rm_simulation rm_simulation.launch.py
ros2 launch sentry_bringup mapping.launch.py 
ros2 run nav2_map_server map_saver_cli -t projected_map -f map --fmt png
```

![](C:\Users\esarchiv\Desktop\map.png)

## 2.ROS2节点

```
ros2 launch pb_rm_simulation rm_simulation.launch.py
ros2 launch sentry_bringup bringup_all_in_one.launch.py
ros2 run tf2_ros static_transform_publisher 4 4 0 0 0 0 map enemy
ros2 run follow follow_node 
ros2 topic echo /tracker/target 
```

## 

## 3.行为树xml

```
ros2 launch rm_decision_cpp run.launch.py
```



## 暂时用不明白