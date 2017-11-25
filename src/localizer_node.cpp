#include "ros/ros.h"
#include "Fusion.h"

#define LOOP_RATE 10

int main(int argc, char **argv) {

    ros::init(argc, argv, "localizer");
    ros::NodeHandle n;

    Fusion pose(n);
    ros::Subscriber navcog_sub = n.subscribe("NavCog/odometry", 1000, &Fusion::NavCogCallback, &pose);
    ros::Subscriber imu_sub = n.subscribe("imu", 1000, &Fusion::IMUCallback, &pose);

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