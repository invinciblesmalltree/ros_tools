#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "ground_camera_node");
    ros::NodeHandle nh;

    // 创建发布器
    ros::Publisher camera_pub =
        nh.advertise<sensor_msgs::Image>("/camera/ground", 10);

    // 打开摄像头设备
    cv::VideoCapture ground_camera("/dev/ground");
    if (!ground_camera.isOpened()) {
        ROS_ERROR("Unable to open ground camera!");
        return -1;
    }

    // 设置摄像头分辨率
    ground_camera.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    ground_camera.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // 设置发布频率为30Hz
    ros::Rate rate(30);

    // 主循环
    while (ros::ok()) {
        cv::Mat frame;
        bool ret = ground_camera.read(frame);

        if (ret) {
            // 转换帧为ROS图像消息
            cv_bridge::CvImage cv_image;
            cv_image.image = frame;
            cv_image.encoding = "bgr8";
            sensor_msgs::Image ros_image;
            cv_image.toImageMsg(ros_image);

            // 发布图像消息
            camera_pub.publish(ros_image);
        }

        rate.sleep();
    }

    // 释放摄像头
    ground_camera.release();

    return 0;
}