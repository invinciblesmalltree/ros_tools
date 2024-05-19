#ifndef ROS_TOOLS_TARGET_CLASS_HPP
#define ROS_TOOLS_TARGET_CLASS_HPP

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros_tools/LidarPose.h>

class target {
  public:
    float x, y, z, yaw;
    bool reached = false;

    target(float x, float y, float z, float yaw) : x(x), y(y), z(z), yaw(yaw) {}

    void fly_to_target(ros::Publisher &local_pos_pub) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = sin(yaw / 2);
        pose.pose.orientation.w = cos(yaw / 2);

        local_pos_pub.publish(pose);
    }

    bool pos_check(ros_tools::LidarPose &lidar_pose_data) {
        if (reached)
            return true;
        return reached = sqrt(pow(lidar_pose_data.x - x, 2) +
                              pow(lidar_pose_data.y - y, 2) +
                              pow(lidar_pose_data.z - z, 2)) < 0.1;
    }
};

#endif // ROS_TOOLS_TARGET_CLASS_HPP