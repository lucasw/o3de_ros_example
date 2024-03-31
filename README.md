# o3de_ros_example

Simple example of using O3DE with ROS

https://github.com/o3de/o3de

https://github.com/o3de/o3de-extras

https://github.com/lucasw/ros_from_src/issues/41

## Building

First time:
```
cmake -B build/linux -S . -G "Ninja Multi-Config"
```

Later builds:

```
source ~/ros/ros2_rolling/install/setup.bash
cd o3de_ros_example/Projects/o3de_ros_project
cmake --build build/linux --target o3de_ros_project.GameLauncher Editor --config profile -j 3
```

This will take a long time the first time, and go really quickly later.

Changing the scene through the editor doesn't require running the build again, only adding a new Gem does, or changing any of the cpp files- I think.
