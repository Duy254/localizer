#include "Fusion.h"

Fusion::Fusion(ros::NodeHandle n) {
    this->publisher = n.advertise<navcog_msg::SimplifiedOdometry>("odometry", 1000);
}

Odom Fusion::getLocation() {
    return this->odom;
}

void Fusion::NavCogCallback(const navcog_msg::SimplifiedOdometry::ConstPtr& msg) {
    // In the future there will be more fancy sensor fusion code
    this->odom.x = msg->pose.x;
    this->odom.y = msg->pose.y;
    this->odom.v = msg->speed;
    this->isUpdated[0] = true;
}

void Fusion::IMUCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
    this->odom.theta = msg->vector.x;
    this->isUpdated[1] = true;
}

void Fusion::publish() {
    navcog_msg::SimplifiedOdometry msg;
    msg.pose.x = this->odom.x;
    msg.pose.y = this->odom.y;
    msg.orientation = this->odom.theta;
    msg.speed = this->odom.v;
    this->publisher.publish(msg);
}

bool Fusion::isAllUpdated() {
    if (!allUpdated){
        allUpdated = true;
        for (auto&& u : isUpdated){
            allUpdated &= u;
        }
    }
    return allUpdated;
}