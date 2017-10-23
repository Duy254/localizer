#ifndef LOCALIZER_LOCATION_H
#define LOCALIZER_LOCATION_H

#include "ros/ros.h"
#include "arduino_msg/Encoder.h"
#include "geometry_msgs/Pose2D.h"

#define ENCODER_FREQ 5 // Note: this should be synchronized with firmware

struct Location {
    float x;
    float y;
};

class Pose{
    Location loc;
    ros::Publisher puber;
public:
    Pose(ros::NodeHandle n);
    Location getLocation();
    void Encoderscallback(const arduino_msg::Encoder::ConstPtr& msg);
    void publish();
};

#endif //LOCALIZER_LOCATION_H
