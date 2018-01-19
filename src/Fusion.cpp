#include "Fusion.h"

Fusion::Fusion(ros::NodeHandle n) {
    this->odom_publisher = n.advertise<nav_msgs::Odometry>("odom", 50);
}

void Fusion::encoderCallback(const arduino_msg::Motor::ConstPtr &msg) {
    // In the future there will be more fancy sensor fusion code
    auto velocity = (msg->left_speed + msg->right_speed) / 2;
    auto distance = velocity / ENCODER_FREQ;
    this->odom.v = velocity;
    this->odom.x += distance * cos(this->odom.theta);
    this->odom.y += distance * sin(this->odom.theta);
    this->isUpdated[0] = true;
}

void Fusion::IMUCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
    auto prev_angle = this->odom.theta;
    this->odom.theta = angles::from_degrees(msg->vector.x);
    this->odom.w = angles::shortest_angular_distance(prev_angle, this->odom.theta) * ENCODER_FREQ;
    this->isUpdated[1] = true;
}

void Fusion::publish() {

    auto current_time = ros::Time::now();

    // Prepare TF transform
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(this->odom.theta);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom"; // Published TF is odom => base_link
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = this->odom.x;
    odom_trans.transform.translation.y = this->odom.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    this->odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";

    //set the position
    odom_msg.pose.pose.position.x = this->odom.x;
    odom_msg.pose.pose.position.y = this->odom.y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    //set the velocity
    odom_msg.child_frame_id = "base_link";
    odom_msg.twist.twist.linear.x = this->odom.v * cos(this->odom.theta);
    odom_msg.twist.twist.linear.y = this->odom.v * sin(this->odom.theta);
    odom_msg.twist.twist.angular.z = this->odom.w;

    //publish the message
    this->odom_publisher.publish(odom_msg);
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