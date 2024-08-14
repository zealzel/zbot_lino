# linorobot2_navigation

## launch_nav

This launch file is for single robot.

Once you provide the worldname argument, the map will be decided.

```bash
ros2 launch linorobot2_navigation launch_nav.py sim:=true rviz:=true worldname:=turtlebot3_house
```

Currently, the world-map relationships is as follows. This maps are in fitrobot/maps/sim directory

- turtlebot3_world: turtlebot3_world.yaml
- room_with_tags: room_with_tags.yaml
- obstacles: obstacles.yaml
