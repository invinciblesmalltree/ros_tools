# -*- coding: utf-8 -*-
import sys
import cv2
import os
import rospy
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
        self.display_width = 640
        self.display_height = 480
        self.frame_rate = 30  # 设置帧率为30

        rospy.init_node("camera_node", anonymous=True)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(int(1000 / self.frame_rate))  # 定时器的间隔与帧率匹配

        self.initUI()

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

        self.record_button = QPushButton("开始录制", self)
        self.record_button.setFont(QFont("Noto Sans CJK SC", 12))  # 设置中文字体
        self.record_button.setCheckable(True)
        self.record_button.clicked.connect(self.toggle_recording)

        vbox = QVBoxLayout()
        hbox = QHBoxLayout()
        hbox.addStretch(1)
        hbox.addWidget(self.label)
        hbox.addStretch(1)
        vbox.addWidget(self.topic_selector)
        vbox.addLayout(hbox)
        vbox.addWidget(self.record_button)

        self.setLayout(vbox)
        self.setWindowTitle("相机节点")
        self.setGeometry(100, 100, 800, 600)
        self.show()

    def update_topics(self):
        self.topic_selector.clear()
        self.topic_selector
