#ifndef LOCALIZER_LOCATION_H
#define LOCALIZER_LOCATION_H

#include "ros/ros.h"
#include <angles/angles.h>
#include <tf/transform_broadcaster.h>
#include <navcog_msg/SimplifiedOdometry.h>
#include <arduino_msg/Motor.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <nav_msgs/Odometry.h>

#include <utility>

// Actually defined in arduino_node, may need to combine these together in the future
#define ENCODER_FREQ 10

namespace cabot {

    typedef std::pair<float, float> Point;

    struct Odom {
        double x;
        double y;
        double theta;
        double v;
        double w; //Actually omega
    };
}

using namespace cabot;

class Fusion{
    Odom odom;
    Point navcog_loc;
    ros::Publisher odom_publisher;
    ros::Publisher initial_pose_publisher;
    tf::TransformBroadcaster odom_broadcaster;
    bool allUpdated = false;
    bool isUpdated[3] = {false, false, false};    
public:
    explicit Fusion(ros::NodeHandle n);

    void NavCogCallback(const navcog_msg::SimplifiedOdometry::ConstPtr &msg);

    void encoderCallback(const arduino_msg::Motor::ConstPtr &msg);
    void IMUCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
    bool isAllUpdated();
    void publish();
};

#endif //LOCALIZER_LOCATION_H
