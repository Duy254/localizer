#ifndef LOCALIZER_LOCATION_H
#define LOCALIZER_LOCATION_H

#include "ros/ros.h"
#include <angles/angles.h>
#include <tf/transform_broadcaster.h>
#include <navcog_msg/SimplifiedOdometry.h>
#include <arduino_msg/Motor.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>

// Actually defined in arduino_node, may need to combine these together in the future
#define ENCODER_FREQ 10

struct Odom {
    double x;
    double y;
    double theta;
    double v;
    double w; //Actually omega
};

class Fusion{
    Odom odom;
    ros::Publisher odom_publisher;
    tf::TransformBroadcaster odom_broadcaster;
    bool allUpdated = false;
    bool isUpdated[2] = {false, false};    
public:
    explicit Fusion(ros::NodeHandle n);

    void encoderCallback(const arduino_msg::Motor::ConstPtr &msg);
    void IMUCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
    bool isAllUpdated();
    void publish();
};

#endif //LOCALIZER_LOCATION_H
