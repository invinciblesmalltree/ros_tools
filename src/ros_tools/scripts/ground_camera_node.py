#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


def ground_camera_node():
    # Initialize the ROS node
    rospy.init_node("ground_camera_node", anonymous=True)

    # Create a publisher for the camera topic
    camera_pub = rospy.Publisher("/camera/ground", Image, queue_size=1)

    # Create a CvBridge object
    bridge = CvBridge()

    # Open the ground camera device
    ground_camera = cv2.VideoCapture("/dev/ground")

    # Set the camera frame width and height
    ground_camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    ground_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Main loop
    rate = rospy.Rate(30)  # 30 Hz
    while not rospy.is_shutdown():
        # Read a frame from the ground camera
        ret, frame = ground_camera.read()

        if ret:
            # Convert the frame to a ROS Image message
            image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

            # Publish the image message
            camera_pub.publish(image_msg)

        rate.sleep()

    # Release the ground camera
    ground_camera.release()


if __name__ == "__main__":
    try:
        ground_camera_node()
    except rospy.ROSInterruptException:
        pass
