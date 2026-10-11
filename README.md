# MS616 — SLAM, Saved-Map Navigation, and Dynamic Obstacle Avoidance

This package provides two navigation workflows:

1. **Recommended: staged navigation.** Map the static environment with dynamic obstacles disabled, save the map, then restart in saved-map localization mode and send goals from a terminal or RViz.
2. **Experimental: online SLAM + frontier exploration.** Continue mapping while Nav2 explores frontiers; an optional LiDAR-based predictor adds short-horizon predictions of moving obstacles.

The staged workflow is easier to debug because mapping and navigation are separate. Use one workflow at a time; do not launch both navigation stacks simultaneously.

## Useful shell commands

```bash
# Leave Conda environments before using the ROS Python environment
conda deactivate

# Workspace
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Raspberry Pi SSH helpers
ping strawberry-desktop.local
ssh strawberry@strawberry-desktop.local

# tmux over SSH: split pane with Ctrl+B then %, detach with Ctrl+B then D
tmux
```

## 1. Build the package

Run from the workspace root:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ms616
source install/setup.bash
```

If `colcon` reports duplicate package names, check that there is only one `ms616` package under `~/ros2_ws/src`. Keep backups outside `src/` so colcon does not discover them as packages:

```bash
colcon list | grep '^ms616 '
```

## 2. Recommended workflow: map once, then navigate using the saved map

### Phase A — automatically explore and map without moving obstacles

No manual driving is needed for normal mapping. The frontier explorer sends goals to Nav2, which moves the robot to the boundary between known free space and unexplored space. The mapping world has no moving obstacle models.

**Terminal 1 — Gazebo + RViz, static world:**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ms616 mapping_gazebo.launch.py
```

**Terminal 2 — EKF + Cartographer + Nav2 + autonomous frontier exploration:**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ms616 mapping.launch.py
```

`autonomous_explore:=true` is the default. The robot explores automatically, while the EKF fuses wheel odometry and IMU, and Cartographer uses the filtered odometry, IMU and LiDAR scans to build `/map`. To debug the stack without automatic exploration, use `autonomous_explore:=false`; this is not the normal workflow.

Wait for the `autonomous_mapper` log saying `No frontiers left — exploration complete.` The map may still be refining for a few seconds; then save it:

```bash
mkdir -p ~/ros2_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/ms616_map
```

This creates `~/ros2_ws/maps/ms616_map.yaml` and `~/ros2_ws/maps/ms616_map.pgm`. Keep both files because the YAML references the image. Once saved, stop the mapping stack with `Ctrl+C` before starting the saved-map navigation workflow.

#### Manual safety override while mapping

Automatic exploration is the default. To temporarily take control, enable the software override first:

```bash
ros2 service call /cmd_vel_mux/set_manual_override std_srvs/srv/SetBool "{data: true}"
```

Then start keyboard teleoperation in a third terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/manual_cmd_vel
```

If the keyboard package is missing, install it once with `sudo apt install ros-humble-teleop-twist-keyboard`. Hold the usual movement keys (`i`, `,`, `j`, `l`, etc. as printed by the teleop node); press `k` to command stop. While override is active, Nav2 velocity commands are blocked, stale keyboard commands time out to zero, the active Nav2 goal is requested to cancel, and `autonomous_mapper` pauses issuing new frontier goals.

To return to automated frontier exploration, stop teleop with `Ctrl+C`, then release the override:

```bash
ros2 service call /cmd_vel_mux/set_manual_override std_srvs/srv/SetBool "{data: false}"
```

The frontier explorer resumes and chooses a goal based on the robot's current pose and map. This is a **software override**, not a certified emergency stop; physical robots still need a hardware E-stop.

### Phase B — start fresh and localize in the saved map

Start a fresh simulator instance; do not leave the mapping launch running.

**Terminal 1 — Gazebo + RViz:**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ms616 gazebo.launch.py dynamic_obstacles:=true rviz:=true
```

Use `dynamic_obstacles:=false` for a static-obstacle baseline. The default world contains the moving obstacles; the manager is enabled only when `dynamic_obstacles:=true`.

**Terminal 2 — saved-map localization + Nav2:**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ms616 saved_map_nav.launch.py \
  map:=$HOME/ros2_ws/maps/ms616_map.yaml
```

This starts:

- the EKF, fusing wheel odometry and IMU to publish filtered local odometry;
- the saved map server;
- AMCL, which uses the current LiDAR scan and saved map to estimate the global robot pose;
- Nav2's planner, controller, behavior server, and BT navigator;
- global and local costmaps that use the saved map and current LiDAR scans.

**Initialize AMCL in RViz:** choose **2D Pose Estimate**, then click and drag at the robot's approximate current pose and heading. Wait for the LiDAR scan to align with the saved-map walls/obstacles before sending a goal. This tool initializes the robot pose; it is not the destination tool.

### Send a goal from the terminal

Get the desired point's coordinates in the `map` frame. In RViz, select **Publish Point**, start this command in a terminal, and click the point on the map:

```bash
ros2 topic echo /clicked_point --once
```

Use the resulting `x` and `y` values. Pick the desired final heading in degrees (`yaw_deg`):

```bash
ros2 launch ms616 goal_from_terminal.launch.py \
  x:=1.5 y:=0.8 yaw_deg:=90
```

Replace the example coordinates and heading with your goal. The coordinates are in **the saved map's `map` frame**, in metres. The launch starts a small goal-sender node; it sends a `NavigateToPose` action to the already-running Nav2 stack. Nav2 itself generates the path and controls the robot.

You can send a goal directly in RViz instead by selecting **Nav2 Goal** (sometimes labelled **2D Goal Pose**) and clicking/dragging to set the destination and desired heading. Do not run the terminal goal sender and an autonomous explorer at the same time.

Useful checks:

```bash
ros2 action list | grep navigate_to_pose
ros2 lifecycle get /controller_server
ros2 lifecycle get /amcl
ros2 topic echo /odometry/filtered --once
ros2 topic echo /scan --once
```

The action `/navigate_to_pose` should exist and `controller_server` / `amcl` should become `active`. If localization looks wrong, reinitialize with **2D Pose Estimate** before changing planner settings.

Manual safety override also works in saved-map navigation. Enable `/cmd_vel_mux/set_manual_override` as described above and run `teleop_twist_keyboard` remapped to `/manual_cmd_vel`. Enabling it requests cancellation of the current Nav2 goal; after releasing the override, send a fresh goal from RViz or the terminal goal sender.


## 3. Experimental workflow: online SLAM + frontier exploration

This is the older workflow where the robot builds a map while Nav2 explores. It is useful for testing frontier exploration and the optional moving-obstacle predictor, but it combines more components and is not the recommended routine navigation mode.

**Terminal 1 — Gazebo:**

```bash
ros2 launch ms616 gazebo.launch.py dynamic_obstacles:=true rviz:=true
```

**Terminal 2 — Cartographer + Nav2 + dynamic prediction + autonomous exploration:**

```bash
ros2 launch ms616 dynamic_nav.launch.py \
  autonomous_explore:=true dynamic_avoidance:=true
```

To run the same live-SLAM stack without autonomous frontier exploration (so you can send goals manually):

```bash
ros2 launch ms616 dynamic_nav.launch.py \
  autonomous_explore:=false dynamic_avoidance:=true
```

To compare against normal LiDAR costmap avoidance without the custom predictor:

```bash
ros2 launch ms616 dynamic_nav.launch.py \
  autonomous_explore:=false dynamic_avoidance:=false
```

When `autonomous_explore:=true`, `autonomous_mapper` sends its own goals. Do not send a manual goal at the same time; disable frontier exploration if testing manual goals.

## 4. What the dynamic-obstacle predictor does

In the online-SLAM workflow, the optional tracker adds a short-horizon motion prediction layer:

```text
/scan
  |
  v
dynamic_obstacle_tracker
  |
  v
/dynamic_obstacles/predicted_points (PointCloud2)
  |
  v
Nav2 local costmap
  |
  v
DWB local controller
  |
  v
/nav_cmd_vel
  |
  v
cmd_vel_mux  <--- /manual_cmd_vel when manual override is enabled
  |
  v
/diff_drive_controller/cmd_vel_unstamped
```

At a high level, the tracker clusters LiDAR returns, associates clusters between scans, estimates their velocities, and extrapolates sufficiently fast tracks into the near future. The predicted points are added to the local costmap so DWB can penalize trajectories that intersect predicted occupancy. Static and currently visible obstacles still come from `/scan`.

The current saved-map launch uses standard Nav2 LiDAR obstacle layers for newly observed obstacles. It does **not** launch the custom velocity-prediction tracker. Add and validate that layer in saved-map mode separately only after saved-map localization and baseline navigation are reliable.

## 5. Obstacle clearance and narrow passages

The robot model's configured `robot_radius` is `0.17 m`, giving a nominal diameter of about `0.34 m`. The planner and costmaps account for obstacle costs and inflation, but the frontier selector and local controller have different responsibilities. A path through a corridor that is narrower than the robot plus clearance margin is not physically feasible; no planner tuning can fix that geometry.

The saved-map global planner is configured with `allow_unknown: false`, so it will not deliberately plan through unknown map cells. The local costmap continuously uses current `/scan` measurements to mark and clear obstacles near the robot.

## 6. Main topics and nodes

For the saved-map workflow, focus on:

```text
/scan
/imu/data_raw
/diff_drive_controller/odom
/odometry/filtered
/map
/tf
/tf_static
/initialpose
/navigate_to_pose
/plan
/local_costmap/costmap
/global_costmap/costmap
/nav_cmd_vel
/manual_cmd_vel
/diff_drive_controller/cmd_vel_unstamped
/cmd_vel_mux/manual_override_active
```

Core nodes include `ekf_filter_node`, `map_server`, `amcl`, `planner_server`, `controller_server`, `behavior_server`, `bt_navigator`, and the lifecycle managers. In online-SLAM mode, also expect `cartographer_node`, `cartographer_occupancy_grid_node`, and optionally `dynamic_obstacle_tracker` and `autonomous_mapper`.

The RQT graph may still look large because ROS 2 actions, lifecycle management, TF, parameters, and standard topics create many edges. For a clearer overview, use node-focused mode and hide `/tf`, `/tf_static`, `/clock`, `/parameter_events`, and `/rosout` when inspecting the graph.

## 7. Important cautions

- Do not run the old independent `obstacle_avoidance.py` / VFH controller at the same time as Nav2. Both can publish velocity commands and fight each other.
- All automated launches route Nav2 commands through `cmd_vel_mux`. Do not remap teleop directly to `/diff_drive_controller/cmd_vel_unstamped`; manual teleop must publish to `/manual_cmd_vel` and the override service must be enabled first.
- Ensure simulation time is enabled for every node in Gazebo/RViz workflows. The launch files default to `use_sim_time:=true`. Avoid starting a second RViz instance without `use_sim_time` enabled.
- Run only one navigation stack at a time. Stop `dynamic_nav.launch.py` before starting `saved_map_nav.launch.py`, and vice versa.
- The standard local costmap reacts to current obstacle observations; predictive moving-obstacle avoidance is only enabled in the online-SLAM `dynamic_nav.launch.py` workflow in this patch.

## 8. Legacy launch files

`explore.launch.py`, `slam_explore.launch.py`, and `controller.launch.py` are retained for compatibility. For the staged workflow use `mapping_gazebo.launch.py` + `mapping.launch.py`, then `gazebo.launch.py` + `saved_map_nav.launch.py`. Use `dynamic_nav.launch.py` only for the experimental live-SLAM/frontier-exploration workflow.
