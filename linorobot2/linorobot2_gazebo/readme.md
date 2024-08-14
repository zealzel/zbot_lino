# linorobot2_gazebo

## worlds

Currently, all simulation worlds are kept in the `worlds` folder under fitrobot package

- room_with_tags.sdf
- turtlebot3_house.world
- turtlebot3_world.world

The default positions for these worlds

- turtlebot3_world: x: 0.5, y: 0.5
- turtlebot3_house: x: -3.0, y: 1.0
- room_with_tags: x: 1.0, y: 1.0

## launch with worldname argument

You can start with worldname without full path. It will use the default positions.

```bash
ros2 launch linorobot2_gazebo launch_sim.py worldname:=room_with_tags
```

## launch with worldpath argument

You can also provide the world full path using worlpath argument

```bash
ros2 launch linorobot2_gazebo launch_sim.py worldpath:=/path/to/file/example_world.world
```

> [notice] Only one of worldname or worldpath can be set at the same time

## launch with customed postions

You can define postions using x & y arguments

```bash
ros2 launch linorobot2_gazebo launch_sim.py worldname:=turtlebot3_world x:=0.5 y:=0.5
```
