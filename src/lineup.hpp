#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#pragma once

class lineup {
   private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;

    std::string inputTopic;
    std::string odomTopic;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;

   public:
    lineup(std::string i, std::string o);
    ~lineup();
    void callback(geometry_msgs::PointStamped inputPoint);
};

lineup::lineup(std::string source, std::string odom) : inputTopic(source), odomTopic(odom) {
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    sub = n.subscribe<geometry_msgs::PointStamped>("/stretch_pc/centerPoint", 1, &lineup::callback, this);
}

lineup::~lineup() {
    delete tfListener;
}

void lineup::callback(geometry_msgs::PointStamped inputPoint) {
    // do something
    std::string transformFrame = "odom";
    geometry_msgs::PointStamped point = tfBuffer.transform(inputPoint, "map");
    geometry_msgs::TransformStamped transOdomToMap = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));

    pub = n.advertise<geometry_msgs::Twist>("/stretch_diff_drive_controller/cmd_vel", 1000);

    tf2::Vector3 v(transOdomToMap.transform.translation.x, transOdomToMap.transform.translation.y, transOdomToMap.transform.translation.z);
    geometry_msgs::Twist rotate;
    rotate.angular.z = 0.5;
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        tfPoint = tfBuffer.transform(inputPoint, "odom");
        ROS_INFO_STREAM(tfPoint);
        pub.publish(rotate);
        loop_rate.sleep();
    }

    geometry_msgs::Vector3 v1;
    tf2::Matrix3x3 m(tf2::Quaternion(transOdomToMap.transform.rotation.x, transOdomToMap.transform.rotation.y, transOdomToMap.transform.rotation.z, transOdomToMap.transform.rotation.w));
    m.getRPY(v1.x, v1.y, v1.z);
    rotate.angular.z = 0;
    pub.publish(rotate);
    ROS_INFO_STREAM("done\n"
                    << point << transOdomToMap);
}