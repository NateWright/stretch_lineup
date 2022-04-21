#include "lineup.hpp"

#include <ros/ros.h>

using std::string;

int main(int argc, char** argv) {
    ros::init(argc, argv, "planar_lineup");
    ros::NodeHandle n;

    string point_topic, cmd_vel, referenceTf, robotTf;
    double offset;
    if (!n.getParam("/stretch_lineup/point_topic", point_topic)) {
        ROS_DEBUG("stretch_lineup: No point topic found\n");
        return (EXIT_FAILURE);
    }
    if (!n.getParam("/stretch_lineup/cmd_vel", cmd_vel)) {
        ROS_DEBUG("stretch_lineup: No cmd vel topic found\n");
        return (EXIT_FAILURE);
    }
    if (!n.getParam("/stretch_lineup/reference_frame_tf", referenceTf)) {
        ROS_DEBUG("stretch_lineup: No refernce frame found\n");
        return (EXIT_FAILURE);
    }
    if (!n.getParam("/stretch_lineup/base_link_tf", robotTf)) {
        ROS_DEBUG("stretch_lineup: No base link transform found\n");
        return (EXIT_FAILURE);
    }
    if (!n.getParam("/stretch_lineup/offset", offset)) {
        ROS_DEBUG("stretch_lineup: No offset found\n");
        return (EXIT_FAILURE);
    }
    ROS_INFO_STREAM("offset: " << offset);
    lineup l(point_topic, cmd_vel, referenceTf, robotTf, offset);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return (EXIT_SUCCESS);
}
