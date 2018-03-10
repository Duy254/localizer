#include "ros/ros.h"
#include "Fusion.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "fake_localizer");
    ros::NodeHandle n("~");

    Fusion pose(n);

    ros::Subscriber command_sub = n.subscribe("/cmd_vel", 1000, &Fusion::commandCallback, &pose);

    ros::Rate loop_rate(SPIN_RATE);
    while (ros::ok())
    {
        pose.spin();
        pose.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();

    return 0;
}