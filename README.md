# MS616 — SLAM + Nav2 + Dynamic Obstacle Avoidance

This package now has one canonical autonomous-navigation entry point:

```bash
ros2 launch ms616 gazebo.launch.py
ros2 launch ms616 dynamic_nav.launch.py
```

The Gazebo launch starts the MS616 robot, LiDAR, IMU, ros2_control, and two
moving collision obstacles. The navigation launch starts Cartographer, a
minimal Nav2 stack, the dynamic-obstacle predictor, and frontier exploration.

## 1. Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ms616
source install/setup.bash
```

## 2. Run Gazebo + moving obstacles

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ms616 gazebo.launch.py
```

By default this starts:

- two dynamic obstacle models in `worlds/default_world.sdf`
- `dynamic_obstacle_manager`, which moves those models through Gazebo's
  `/world/ms616_default/set_pose` service
- the normal `/scan` and `/imu/data_raw` bridges

Useful options:

```bash
# Start without moving obstacles
ros2 launch ms616 gazebo.launch.py dynamic_obstacles:=false

# Start RViz from the Gazebo launch as well
ros2 launch ms616 gazebo.launch.py rviz:=true
```

Verify the simulator side first:

```bash
ros2 topic echo /scan --once
ros2 service list | grep /world/ms616_default/set_pose
```

## 3. Run navigation

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ms616 dynamic_nav.launch.py
```

For mapping / Nav2 without autonomous frontier exploration:

```bash
ros2 launch ms616 dynamic_nav.launch.py autonomous_explore:=false
```

To disable the prediction layer and compare against plain Nav2 local-costmap
avoidance:

```bash
ros2 launch ms616 dynamic_nav.launch.py dynamic_avoidance:=false
```

## 4. What the new dynamic-avoidance layer does

The implementation is inspired by the linked Nav2 dynamic-obstacle project:

```text
                 /scan
                   |
                   v
        dynamic_obstacle_tracker
                   |
                   | predicted PointCloud2
                   v
      /dynamic_obstacles/predicted_points
                   |
                   v
          Nav2 local costmap
                   |
                   v
               DWB controller
                   |
                   v
      /diff_drive_controller/cmd_vel_unstamped
```

The tracker adds short-horizon prediction rather than only reacting to the
obstacle's current LiDAR position:

1. LiDAR returns are segmented into compact clusters.
2. Cluster centroids are transformed into `odom`.
3. Clusters are associated between scans.
4. A filtered velocity estimate is formed for each track.
5. Tracks moving faster than the configured threshold are extrapolated about
   1.1 s into the future.
6. The predicted occupied tube is published as `PointCloud2`.
7. The Nav2 local costmap marks those predicted points.
8. DWB rejects / penalizes candidate trajectories that enter the predicted
   occupied region.

Static obstacles still come directly from `/scan`.

## 5. Main parameters

The important tuning is in `config/nav2_params.yaml` and the
`dynamic_nav.launch.py` arguments.

Tracker defaults:

- prediction horizon: 1.10 s
- dynamic-speed threshold: 0.10 m/s
- prediction lateral radius: 0.12 m
- association distance: 0.50 m
- track timeout: 0.60 s

Local costmap gets two obstacle sources:

```text
scan                -> current LiDAR obstacles
 dynamic_prediction -> predicted moving obstacles
```

The global costmap remains scan + static map only. That is intentional: the
moving object should not be permanently written into the global map.

## 6. Why the RQT graph is smaller now

Do not launch the old independent reactive controller alongside Nav2. The old
`obstacle_avoidance.py` publishes its own velocity command and will fight the
Nav2 controller if both are active.

The canonical graph is approximately:

```text
Gazebo / ros2_control
        |
        +--> /scan ------> Cartographer ------> /map
        |       |
        |       +-------> dynamic_obstacle_tracker
        |                         |
        |                         v
        |                predicted obstacles
        |                         |
        +--> /odom --------------+----> Nav2 local costmap
                                          |
/map ------------------------------> Nav2 planner
                                          |
                                   global path + costmap
                                          |
                                       DWB
                                          |
                                          v
                             diff_drive_controller
```

The important navigation nodes are:

```text
cartographer_node
cartographer_occupancy_grid_node
dynamic_obstacle_tracker
planner_server
controller_server
behavior_server
bt_navigator
lifecycle_manager_navigation
autonomous_mapper   # only for frontier exploration
```

The optional `smoother_server`, `waypoint_follower`, and `velocity_smoother`
are intentionally not launched by `dynamic_nav.launch.py` because this project
only needs `NavigateToPose` and a differential-drive command path.

For a readable RQT graph, use `rqt_graph` in node-focused mode and hide
standard plumbing topics such as `/tf`, `/tf_static`, `/clock`,
`/parameter_events`, and `/rosout`.

## 7. Manual navigation test

With `autonomous_explore:=false`, you can send a test goal directly through
Nav2's action interface or from RViz.

For example, inspect the action server with:

```bash
ros2 action list | grep navigate_to_pose
```

Then use RViz's Nav2 Goal tool.

## 8. Legacy launch names

`explore.launch.py`, `slam_explore.launch.py`, and `controller.launch.py` are
kept only as compatibility wrappers. Use `dynamic_nav.launch.py` for the
actual navigation stack.
