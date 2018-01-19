# localizer

This node acts as the entry point for all the localization stuff of Cabot. Currently it only calculates odometry.

## What's subscribed and published

- Subscribed
    - `/encoder`: Encoder information from Arduino
    - `/imu`: IMU information from Arduino
- Published
    - `/odom`: a `nav_msgs::Odometry` message, containing fused odometry
    - `tf` transform from `/odom` to `base_link`