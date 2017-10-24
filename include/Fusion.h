#ifndef LOCALIZER_LOCATION_H
#define LOCALIZER_LOCATION_H

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"

struct Pose {
    double x;
    double y;
};

class Fusion{
    Pose pos;
    ros::Publisher puber;
public:
    Fusion(ros::NodeHandle n);
    Pose getLocation();
    void NavCogCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
    void publish();
};

#endif //LOCALIZER_LOCATION_H
