# localizer

This node acts as the entry point for all the localization stuff of Cabot.

The source code includes odometry calculation. Particle filter localization is included in the launch file

## Overall localization

Just connect Arduino and Lidar to your computer, then run
```
roslaunch localizer localize.launch
```
You can see all the things in `rviz`. The `rviz` configuration will be updated in the next commit.

## Odom calculation

You can see the demo of how to use this node in `get_odom.launch`.

- Subscribed
    - `/encoder`: Encoder information from Arduino
    - `/imu`: IMU information from Arduino
- Published
    - `/odom`: a `nav_msgs::Odometry` message, containing fused odometry
    - `tf` transform from `/odom` to `base_link`