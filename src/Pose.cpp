#include "Pose.h"

Pose::Pose(ros::NodeHandle n) {
    this->puber = n.advertise<geometry_msgs::Pose2D>("pose", 1000);
}

Location Pose::getLocation() {
    return this->loc;
}

void Pose::Encoderscallback(const arduino_msg::Encoder::ConstPtr& msg) {
    this->loc.x+=msg->speed*1/ENCODER_FREQ;
}

void Pose::publish() {
    geometry_msgs::Pose2D msg;
    msg.x = this->loc.x;
    msg.y = this->loc.y;
    msg.theta = 0;
    this->puber.publish(msg);
}