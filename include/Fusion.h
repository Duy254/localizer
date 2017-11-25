#ifndef LOCALIZER_LOCATION_H
#define LOCALIZER_LOCATION_H

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <navcog_msg/SimplifiedOdometry.h>
#include <arduino_msg/Motor.h>
#include <geometry_msgs/Vector3Stamped.h>

struct Odom {
    double x;
    double y;
    double z;
    double theta;
    double v;
    double w; //Actually omega
};

class Fusion{
    Odom odom;
    ros::Publisher publisher;
    bool allUpdated;
    bool isUpdated[2] = {false, false};    
public:
    explicit Fusion(ros::NodeHandle n);
    Odom getLocation();
    void NavCogCallback(const navcog_msg::SimplifiedOdometry::ConstPtr& msg);
    void IMUCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
    bool isAllUpdated();
    void publish();
};

#endif //LOCALIZER_LOCATION_H
