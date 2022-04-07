#include "lineup.hpp"

#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "planar_lineup");
    ros::NodeHandle n;

    lineup l("/stretch_pc/centerPoint", "odom");

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return (EXIT_SUCCESS);
}
