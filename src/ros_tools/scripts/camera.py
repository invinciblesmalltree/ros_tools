# -*- coding: utf-8 -*-
import sys
import cv2
import os
import rospy
import time
import threading
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtWidgets import (
    QApplication,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QWidget,
    QComboBox,
    QLineEdit,
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap, QFont


class CameraNode(QWidget):
    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()
        self.subscriber = None
        self.image = None
        self.recording = False
        self.video_writer = None
        self.video_counter = 1
        self.photo_counter = 1
        self.display_width = 640
        self.display_height = 480
        self.frame_rate = 30  # 默认帧率为30
        self.recording_thread = None
        self.initUI()

        rospy.init_node("camera_node", anonymous=True)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(int(1000 / self.frame_rate))  # 定时器的间隔与帧率匹配

    def initUI(self):
        self.label = QLabel(self)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setFixedSize(self.display_width, self.display_height)
        self.label.setFont(QFont("Noto Sans CJK SC", 12))  # 设置中文字体

        self.topic_selector = QComboBox(self)
        self.topic_selector.addItem("选择ROS话题")
        self.topic_selector.setFont(QFont("Noto Sans CJK SC", 12))  # 设置中文字体
        self.topic_selector.activated[str].connect(self.change_topic)
        self.topic_selector.showPopup = self.update_topics  # 重新定义showPopup方法

        self.frame_rate_input = QLineEdit(self)
        self.frame_rate_input.setText("30")  # 默认输入值为30
        self.frame_rate_input.setFont(QFont("Noto Sans CJK SC", 12))  # 设置中文字体
        self.frame_rate_input.setFixedWidth(120)

        self.record_button = QPushButton("开始录制", self)
        self.record_button.setFont(QFont("Noto Sans CJK SC", 12))  # 设置中文字体
        self.record_button.setCheckable(True)
        self.record_button.clicked.connect(self.toggle_recording)

        self.photo_button = QPushButton("拍照", self)
        self.photo_button.setFont(QFont("Noto Sans CJK SC", 12))  # 设置中文字体
        self.photo_button.clicked.connect(self.take_photo)

        vbox = QVBoxLayout()
        hbox_label = QHBoxLayout()
        hbox_label.addStretch(1)
        hbox_label.addWidget(self.label)
        hbox_label.addStretch(1)

        hbox_bottom = QHBoxLayout()
        hbox_bottom.addWidget(self.frame_rate_input)
        hbox_bottom.addWidget(self.record_button)
        hbox_bottom.addWidget(self.photo_button)

        vbox.addWidget(self.topic_selector)
        vbox.addLayout(hbox_label)
        vbox.addLayout(hbox_bottom)

        self.setLayout(vbox)
        self.setWindowTitle("相机节点")
        self.setGeometry(100, 100, 800, 600)
        self.show()

    def update_topics(self):
        self.topic_selector.clear()
        self.topic_selector.addItem("选择ROS话题")
        topics = rospy.get_published_topics()
        for topic, msg_type in topics:
            if msg_type == "sensor_msgs/Image":
                self.topic_selector.addItem(topic)
        QComboBox.showPopup(self.topic_selector)

    def change_topic(self, topic):
        if topic != "选择ROS话题":
            if self.subscriber:
                self.subscriber.unregister()
            self.subscriber = rospy.Subscriber(topic, Image, self.image_callback)

    def image_callback(self, data):
        try:
            if data.encoding == "32FC1":
                cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
                cv_image = np.nan_to_num(cv_image, nan=0.0)
                normalized_image = cv2.normalize(
                    cv_image, None, 0, 255, cv2.NORM_MINMAX
                )
                self.image = cv2.convertScaleAbs(normalized_image)
            else:
                self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def update_frame(self):
        if self.image is not None:
            resized_image = cv2.resize(
                self.image, (self.display_width, self.display_height)
            )
            if len(resized_image.shape) == 2:  # 如果是灰度图像
                qImg = QImage(
                    resized_image.data,
                    resized_image.shape[1],
                    resized_image.shape[0],
                    QImage.Format_Grayscale8,
                )
            else:  # 如果是彩色图像
                height, width, channel = resized_image.shape
                bytes_per_line = 3 * width
                qImg = QImage(
                    resized_image.data,
                    width,
                    height,
                    bytes_per_line,
                    QImage.Format_RGB888,
                ).rgbSwapped()
            self.label.setPixmap(QPixmap.fromImage(qImg))

    def toggle_recording(self):
        if self.record_button.isChecked():
            self.record_button.setText("停止录制")
            self.start_recording()
        else:
            self.record_button.setText("开始录制")
            self.stop_recording()

    def start_recording(self):
        try:
            self.frame_rate = int(self.frame_rate_input.text())
        except ValueError:
            self.frame_rate = 30  # 使用默认帧率30
        filename = f"video_{self.video_counter}.mp4"
        while os.path.exists(filename):
            self.video_counter += 1
            filename = f"video_{self.video_counter}.mp4"
        self.video_writer = cv2.VideoWriter(
            filename,
            cv2.VideoWriter_fourcc(*"mp4v"),
            self.frame_rate,
            (self.image.shape[1], self.image.shape[0]),
        )
        self.recording = True
        self.recording_thread = threading.Thread(target=self.record_video)
        self.recording_thread.start()

    def stop_recording(self):
        self.recording = False
        if self.recording_thread:
            self.recording_thread.join()
        if self.video_writer:
            self.video_writer.release()
        self.video_writer = None
        self.video_counter += 1

    def record_video(self):
        while self.recording:
            if self.image is not None:
                self.video_writer.write(self.image)
            time.sleep(1.0 / self.frame_rate)

    def take_photo(self):
        if self.image is not None:
            filename = f"photo_{self.photo_counter}.png"
            while os.path.exists(filename):
                self.photo_counter += 1
                filename = f"photo_{self.photo_counter}.png"
            cv2.imwrite(filename, self.image)
            self.photo_counter += 1


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = CameraNode()
    sys.exit(app.exec_())
