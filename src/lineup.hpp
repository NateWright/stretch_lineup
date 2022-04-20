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

// void lineup::callback(geometry_msgs::PointStamped inputPoint) {
//     // do something
//     std::string transformFrame = "odom";
//     geometry_msgs::PointStamped point = tfBuffer.transform(inputPoint, "map");
//     geometry_msgs::TransformStamped transOdomToMap = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));

//     pub = n.advertise<geometry_msgs::Twist>("/stretch_diff_drive_controller/cmd_vel", 1000);

//     tf2::Vector3 v(transOdomToMap.transform.translation.x, transOdomToMap.transform.translation.y, transOdomToMap.transform.translation.z);
//     geometry_msgs::Twist rotate;
//     rotate.angular.z = 0.5;
//     ros::Rate loop_rate(10);
//     while (ros::ok()) {
//         tfPoint = tfBuffer.transform(inputPoint, "odom");
//         ROS_INFO_STREAM(tfPoint);
//         pub.publish(rotate);
//         loop_rate.sleep();
//     }

//     geometry_msgs::Vector3 v1;
//     tf2::Matrix3x3 m(tf2::Quaternion(transOdomToMap.transform.rotation.x, transOdomToMap.transform.rotation.y, transOdomToMap.transform.rotation.z, transOdomToMap.transform.rotation.w));
//     m.getRPY(v1.x, v1.y, v1.z);
//     rotate.angular.z = 0;
//     pub.publish(rotate);
//     ROS_INFO_STREAM("done\n"
//                     << point << transOdomToMap);
// }

double getYaw(const geometry_msgs::Quaternion& input) {
    tf2::Quaternion q;
    tf2::fromMsg(input, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    return yaw;
}

void lineup::callback(geometry_msgs::PointStamped inputPoint) {
    std::string targetFrame = "map";
    std::string sourceFrame = "base_link";
    geometry_msgs::PointStamped point = tfBuffer.transform(inputPoint, targetFrame);
    geometry_msgs::TransformStamped transBaseLinkToMap = tfBuffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0));
    double x = transBaseLinkToMap.transform.translation.x - point.point.x;
    double y = transBaseLinkToMap.transform.translation.y - point.point.y;
    double h = sqrt(x * x + y * y);
    ROS_INFO_STREAM(std::endl
                    << "point: " << std::endl
                    << "x: " << point.point.x << std::endl
                    << "y: " << point.point.y);
    ROS_INFO_STREAM(std::endl
                    << "translation: " << std::endl
                    << "x: " << transBaseLinkToMap.transform.translation.x << std::endl
                    << "y: " << transBaseLinkToMap.transform.translation.y);
    ROS_INFO_STREAM(std::endl
                    << "translation - point: " << std::endl
                    << "x: " << x << std::endl
                    << "y: " << y << std::endl
                    << "h: " << h);
    double angle = acos(y / h);
    // double angle = acos(y / h) - acos(transBaseLinkToMap.transform.translation.y / sqrt(transBaseLinkToMap.transform.translation.x * transBaseLinkToMap.transform.translation.x + transBaseLinkToMap.transform.translation.y * transBaseLinkToMap.transform.translation.y));
    ROS_INFO_STREAM(std::endl
                    << "acos: " << angle);
    // ROS_INFO_STREAM(std::endl
    //                 << "angle: " << angle);

    double yaw = getYaw(transBaseLinkToMap.transform.rotation);

    geometry_msgs::Twist rotate;
    rotate.angular.z = 0.5;
    ros::Rate loop_rate(10);
    pub = n.advertise<geometry_msgs::Twist>("/stretch_diff_drive_controller/cmd_vel", 1000);
    // ROS_INFO_STREAM(point << std::endl
    //                       << transBaseLinkToMap);
    ROS_INFO_STREAM(fabs(yaw - angle));
    while (ros::ok() && fabs(yaw - angle) > 0.10) {
        transBaseLinkToMap = tfBuffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0));
        yaw = getYaw(transBaseLinkToMap.transform.rotation);
        ROS_INFO_STREAM(fabs(yaw - angle));

        if (yaw - angle < 0) {
            rotate.angular.z = 0.5;
        } else {
            rotate.angular.z = -0.5;
        }
        pub.publish(rotate);
        loop_rate.sleep();
    }
}