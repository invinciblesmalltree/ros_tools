#ifndef ROS_TOOLS_TARGET_CLASS_HPP
#define ROS_TOOLS_TARGET_CLASS_HPP

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros_tools/LidarPose.h>

class target {
  public:
    float x, y, z, yaw;
    bool reached = false;

    geometry_msgs::PoseStamped pose;

    target(float x, float y, float z, float yaw) : x(x), y(y), z(z), yaw(yaw) {}

    void fly_to_target(ros::Publisher &local_pos_pub) {
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = sin(yaw / 2);
        pose.pose.orientation.w = cos(yaw / 2);

        local_pos_pub.publish(pose);
    }

    bool pos_check(ros_tools::LidarPose &lidar_pose_data,
                   double distance) { // 简约的写法
        return reached ||
               (reached = sqrt(pow(lidar_pose_data.x - x, 2) +
                               pow(lidar_pose_data.y - y, 2) +
                               pow(lidar_pose_data.z - z, 2)) < distance) &&
                   abs(lidar_pose_data.yaw - yaw) < 0.2;
    }

    bool pos_check(ros_tools::LidarPose &lidar_pose_data) {
        return reached || pos_check(lidar_pose_data, 0.1);
    }

    bool pos_check(ros_tools::LidarPose &lidar_pose_data, double distance_x,
                   double distance_y, double distance_z) {
        return reached || (reached = abs(lidar_pose_data.x - x) < distance_x &&
                                     abs(lidar_pose_data.y - y) < distance_y &&
                                     abs(lidar_pose_data.z - z) < distance_z) &&
                              abs(lidar_pose_data.yaw - yaw) < 0.2;
    }
};

#endif // ROS_TOOLS_TARGET_CLASS_HPP