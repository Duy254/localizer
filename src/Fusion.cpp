#include "Fusion.h"

Fusion::Fusion(ros::NodeHandle n) {
    this->puber = n.advertise<geometry_msgs::Pose2D>("pose", 1000);
}

Pose Fusion::getLocation() {
    return this->pos;
}

void Fusion::NavCogCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
    // In the future there will be more fancy sensor fusion code
    this->pos.x = msg->x;
    this->pos.y = msg->y;
}

void Fusion::publish() {
    geometry_msgs::Pose2D msg;
    msg.x = this->pos.x;
    msg.y = this->pos.y;
    msg.theta = 0;
    this->puber.publish(msg);
}