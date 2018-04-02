# localizer

This node acts as the entry point for all the localization stuff of Cabot.

The source code includes odometry calculation. Particle filter localization is included in the launch file

## How to see planner working in simulation

```
roslaunch localizer simulated_planner.launch
```

You can dynamically change all the parameters in the opened `rqt_reconfigre` window.

The most important thing to tune is the local planner part. Feel free to play around using `rqt_reconfigure` and modify it in `base_local_planner_params.yaml`

You can switch between `dwa_local_planner` and `teb_local_planner` or the default `base_local_planner` in `configs/move_base_params.yaml`.

## How to send goal and see Cabot run

```
roslaunch localizer move_base.launch
```

Then set goals in `rviz`

## Overall localization

First, install dependencies:

```
sudo apt-get install ros-kinetic-navigation
```

Just connect Arduino and Lidar to your computer, then run
```
roslaunch localizer localize.launch
```

### What's actually happening

If you look at `localize.launch`, I do these things:
- Read sensor readings from Arduino and lidar
- Calculate odometry base on encoder and IMU, and publish the corresponding TF transform `/odom` -> `base_link`
- publish static TF transforms by calling `robot_state_publisher` and `urdf_publisher`
- Publish the map by launching `map_server`
- Localize using lidar reading and odometry using `amcl`. Publish the transform `/map` -> `odom`, also publish `amcl_pose`
- Use `hector_tragetory_server` to generate the robot tragetory
- Launch `rviz` for a nice UI

## Odom calculation

You can see the demo of how to use this node in `get_odom.launch`.

- Subscribed
    - `/encoder`: Encoder information from Arduino
    - `/imu`: IMU information from Arduino
- Published
    - `/odom`: a `nav_msgs::Odometry` message, containing fused odometry
    - `tf` transform from `/odom` to `/base_link`
