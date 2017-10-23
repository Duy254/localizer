#include "ros/ros.h"
#include "Pose.h"

#define LOOP_RATE 10

int main(int argc, char **argv) {

    ros::init(argc, argv, "localizer");
    ros::NodeHandle n;

    Pose pose(n);
    ros::Subscriber encoder_sub = n.subscribe("encoder", 1000, &Pose::Encoderscallback, &pose);

    ros::Rate loop_rate(LOOP_RATE);
    while (ros::ok())
    {
        pose.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();

    return 0;
}