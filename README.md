# ms616

# Package Name: 
``` ms616```

### setting up the terminal
```
conda deactivate
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws

```


### Running the bot in gazebo after launching 
gazebo.launch.py:

```
ros2 topic pub --rate 10 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

### launching the overall setup : 

```
ros2 launch gazebo.launch.py
``` 
