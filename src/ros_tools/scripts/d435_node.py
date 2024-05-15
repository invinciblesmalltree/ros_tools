#!/usr/bin/env python
import rospy
import pyrealsense2 as rs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


def main():
    rospy.init_node("d435_node", anonymous=True)

    rgb_pub = rospy.Publisher("/d435/rgb", Image, queue_size=10)
    depth_pub = rospy.Publisher("/d435/depth", Image, queue_size=10)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    pipeline.start(config)

    bridge = CvBridge()

    try:
        while not rospy.is_shutdown():
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            rgb_msg = bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")

            rgb_pub.publish(rgb_msg)
            depth_pub.publish(depth_msg)

            rospy.sleep(0.1)
    finally:
        pipeline.stop()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
