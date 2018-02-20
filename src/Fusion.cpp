#include "Fusion.h"

static Point navcog2map(Point navcog_point);

// Euclidian distance between the two localization system
static double distance(Point navcog_loc, Odom odom);

Fusion::Fusion(ros::NodeHandle n) {
    this->odom_publisher = n.advertise<nav_msgs::Odometry>("/odom", 50);
    this->initial_pose_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 5);
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

void Fusion::NavCogCallback(const navcog_msg::SimplifiedOdometry::ConstPtr &msg) {
    Point navcog_point(msg->pose.x, msg->pose.y);
    this->navcog_loc = navcog2map(navcog_point); // Only trust x and y data
    // Check whether the position Navcog gives is too far off from that from amcl
    if (distance(this->navcog_loc, this->odom) > DRIFT_TOLERENCE) {
        if (!this->isUpdated[2]){
            ROS_INFO_STREAM("Received Navcog Data, initialize location to: ("<<this->navcog_loc.first<<", "<<this->navcog_loc.second<<")");
        } else {
        ROS_WARN_STREAM("Main localization system drift too much, switched to Navcog location.");
        ROS_INFO_STREAM("Odom: { x: " << this->odom.x << ", y: " << this->odom.y << " }, Navcog: { x: "
                                      << this->navcog_loc.first << ", y: " << this->navcog_loc.second << " }");
        }
        this->odom.x = this->navcog_loc.first;
        this->odom.y = this->navcog_loc.second;
        // Publish (re)initialize pose for amcl
        geometry_msgs::PoseWithCovarianceStamped initial_pose_msg;

        initial_pose_msg.header.stamp = ros::Time::now();
        initial_pose_msg.header.frame_id = "odom";

        initial_pose_msg.pose.pose.position.x = this->odom.x;
        initial_pose_msg.pose.pose.position.y = this->odom.y;
        initial_pose_msg.pose.pose.position.z = 0;

        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(this->odom.theta);
        initial_pose_msg.pose.pose.orientation = quat;

        this->initial_pose_publisher.publish(initial_pose_msg);
    }

    this->isUpdated[2] = true;
}

void Fusion::commandCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    this->odom.v = msg->linear.x;
    this->odom.w = msg->angular.z;
    ROS_INFO_STREAM(this->odom.w);
}

void Fusion::spin() {
    this->odom.theta += this->odom.w / SPIN_RATE;
    auto distance = this->odom.v / SPIN_RATE;
    this->odom.x += distance * cos(this->odom.theta);
    this->odom.y += distance * sin(this->odom.theta);
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

void Fusion::forgetNavcog() {
    this->isUpdated[2] = true;
}

static Point navcog2map(Point navcog_point) {
    navcog_point.first = -navcog_point.first - 9;
    navcog_point.second = -navcog_point.second;
    return navcog_point;
}

static double distance(Point navcog_loc, Odom odom) {
    return sqrt(pow(navcog_loc.first - odom.x, 2) + pow(navcog_loc.second - odom.y, 2));
}