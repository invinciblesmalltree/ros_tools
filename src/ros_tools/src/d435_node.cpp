#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "d435_node");
    ros::NodeHandle nh;

    ros::Publisher rgb_pub = nh.advertise<sensor_msgs::Image>("/d435/rgb", 1);
    ros::Publisher depth_pub =
        nh.advertise<sensor_msgs::Image>("/d435/depth", 1);

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);

    rs2::pipeline_profile selection = pipe.start(cfg);

    cv_bridge::CvImagePtr cv_ptr_rgb(new cv_bridge::CvImage);
    cv_bridge::CvImagePtr cv_ptr_depth(new cv_bridge::CvImage);

    cv_ptr_rgb->encoding = sensor_msgs::image_encodings::BGR8;
    cv_ptr_depth->encoding = sensor_msgs::image_encodings::TYPE_16UC1;

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();
        rs2::frame depth_frame = frames.get_depth_frame();

        if (!color_frame || !depth_frame)
            continue;

        cv::Mat color_image(cv::Size(1280, 720), CV_8UC3,
                            (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth_image(cv::Size(1280, 720), CV_16UC1,
                            (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        cv_ptr_rgb->image = color_image;
        cv_ptr_depth->image = depth_image;

        sensor_msgs::ImagePtr rgb_msg = cv_ptr_rgb->toImageMsg();
        sensor_msgs::ImagePtr depth_msg = cv_ptr_depth->toImageMsg();

        rgb_pub.publish(rgb_msg);
        depth_pub.publish(depth_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    pipe.stop();
    return 0;
}