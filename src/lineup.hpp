#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#pragma once

class lineup {
   private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;

    std::string targetFrame;
    std::string sourceFrame;

    double offset;

   public:
    lineup(std::string inputPointTopic, std::string cmdVelTopic, std::string frameOfReference, std::string base_link, double offset);
    ~lineup();
    void callback(geometry_msgs::PointStamped inputPoint);
};

lineup::lineup(std::string inputPointTopic, std::string cmdVelTopic, std::string frameOfReference, std::string base_link, double offset)
    : targetFrame(frameOfReference), sourceFrame(base_link), offset(offset) {
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    sub = n.subscribe<geometry_msgs::PointStamped>(inputPointTopic, 1, &lineup::callback, this);
    pub = n.advertise<geometry_msgs::Twist>(cmdVelTopic, 1000);
}

lineup::~lineup() { delete tfListener; }

void lineup::callback(geometry_msgs::PointStamped inputPoint) {
    geometry_msgs::PointStamped point = tfBuffer.transform(inputPoint, targetFrame);
    geometry_msgs::TransformStamped transBaseLinkToMap = tfBuffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0));
    double x = point.point.x - transBaseLinkToMap.transform.translation.x;
    double y = point.point.y - transBaseLinkToMap.transform.translation.y;
    double angle = atan2(y, x);

    double yaw = tf2::getYaw(transBaseLinkToMap.transform.rotation) + offset;

    geometry_msgs::Twist rotate;
    rotate.angular.z = 0.5;
    ros::Rate loop_rate(10);

    while (ros::ok() && fabs(angle - yaw) > 0.10) {
        transBaseLinkToMap = tfBuffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0));
        yaw = tf2::getYaw(transBaseLinkToMap.transform.rotation) + offset;

        if (yaw - angle < 0) {
            rotate.angular.z = 0.5;
        } else {
            rotate.angular.z = -0.5;
        }
        pub.publish(rotate);
        loop_rate.sleep();
    }
}