import sys
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QHBoxLayout, QWidget, QVBoxLayout, QSlider, QCheckBox, QPushButton, QLineEdit, QSizePolicy
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPixmap, QImage, QCursor
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import signal
from functools import partial
from interfaces.srv import GetCurrentMap, AddMap
from functools import partial
import numpy as np
import cv2 as cv
import subprocess
from pathlib import Path


class Ros2BagAPI():
    def __init__(self, recording_topics=None):
        self.recording_topics = recording_topics
        self.process = None
        self.bag_file_name = None

        self.playback_remapping_old_topic = None
        self.playback_remapping_new_topic = None

        self.playback_rate = 1.0

    def start_recording(self):
        self.process = subprocess.Popen(
            ["ros2", "bag", "record", *self.recording_topics, "-o", self.bag_file_name, "--compression-mode", "message", "--compression-format", "zstd"])

    def stop_recording(self):
        if self.process:
            self.process.terminate()
            # self.process.wait()
        else:
            print('No recording to stop.')

    def start_playback(self):
        self.process = subprocess.Popen(
            ["ros2", "bag", "play", self.bag_file_name, "--remap", f"{self.playback_remapping_old_topic}:={self.playback_remapping_new_topic}", "--rate", str(self.playback_rate)])

    def stop_playback(self):
        if self.process:
            self.process.terminate()
            # self.process.wait()
        else:
            print('No playback to stop.')

    def set_bag_file_name(self, text):
        self.bag_file_name = text

    def set_playback_remapping_old_topic(self, text):
        self.playback_remapping_old_topic = text

    def set_playback_remapping_new_topic(self, text):
        self.playback_remapping_new_topic = text

    def set_playback_rate(self, rate):
        self.playback_rate = rate/10

    def shutdown(self):
        if self.process:
            self.process.terminate()
            self.process.wait()


class ImageViewer(QLabel):
    def __init__(self, robot_name, node):
        super().__init__()

        self.node = node

        self.subscription = self.node.create_subscription(
            Image, f'{robot_name}/camera/image_color', self.image_callback, 5)
        self.cmd_vel_publisher = self.node.create_publisher(
            Twist, f'{robot_name}/cmd_vel', 10)

        self.initial_mouse_pos = None
        self.twist = Twist()

        self.speed = 1.0
        self.allowTilt = False

        self.display_image(None)

    def image_callback(self, msg):
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv.resize(
                cv_image, (int(self.geometry().width()-10), int((240/320)*(self.geometry().width()-10))))
            # Convert the OpenCV image to QImage
            height, width, channels = cv_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(cv_image.data, width, height,
                             bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            # Display the QImage using PyQt
            self.display_image(q_image)

        except Exception as e:
            self.node.get_logger().error(f'Error processing the image: {e}')

    def display_image(self, image):
        if image is None:  # Check if the image is null or empty
            # Create a black image
            black_image = np.zeros((240, 320, 3), dtype=np.uint8)
            q_image = QImage(black_image.data, 320, 240, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_image)
            self.setPixmap(pixmap)
        else:
            # Display the image using PyQt
            pixmap = QPixmap.fromImage(image)
            self.setPixmap(pixmap)

    def mousePressEvent(self, event):
        self.setFocus(True)

        self.initial_mouse_pos = event.pos()

    def mouseReleaseEvent(self, event):
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

        self.cmd_vel_publisher.publish(self.twist)

    def mouseMoveEvent(self, event):
        offset = event.pos() - self.initial_mouse_pos

        if self.allowTilt:
            self.twist.angular.y = offset.y()/100

        self.twist.angular.z = -offset.x()/100

        self.cmd_vel_publisher.publish(self.twist)

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_W:
            self.twist.linear.x = self.speed
        elif key == Qt.Key_A:
            self.twist.linear.y = self.speed
        elif key == Qt.Key_S:
            self.twist.linear.x = -self.speed
        elif key == Qt.Key_D:
            self.twist.linear.y = -self.speed

        self.cmd_vel_publisher.publish(self.twist)

    def keyReleaseEvent(self, event):
        key = event.key()
        if key == Qt.Key_W or key == Qt.Key_S:
            self.twist.linear.x = 0.0
        elif key == Qt.Key_A or key == Qt.Key_D:
            self.twist.linear.y = 0.0

        self.cmd_vel_publisher.publish(self.twist)

    def updateSpeed(self, speed):
        self.speed = float(speed)

    def setAllowTilt(self, allow):
        self.allowTilt = allow


class MainWindow(QMainWindow):
    speed = 1.0

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.robot_names = ["robot1", "robot2"]

        # ROS2 init
        rclpy.init(args=None)
        self.node = Node('image_subscriber_node')
        # spin once, timeout_sec 5[s]
        timeout_sec_rclpy = 5
        rclpy.spin_once(self.node, timeout_sec=timeout_sec_rclpy)
        print("Connected to ros")

        self.robot_viewers = []

        ### Create interface ###
        self.setWindowTitle('Image Viewer')
        central_widget = QWidget()

        layout = QVBoxLayout()

        # Robot viewers
        robot_viewers_layout = QHBoxLayout()
        for robot_name in self.robot_names:
            robot_viewer = ImageViewer(robot_name, self.node)
            self.robot_viewers.append(robot_viewer)

            robot_viewers_layout.addWidget(robot_viewer)

        layout.addLayout(robot_viewers_layout)

        # Speed slider
        speed_layout = QHBoxLayout()
        self.speed_label = QLabel("Speed: 1")
        speed_layout.addWidget(self.speed_label)
        speed_slider = QSlider(Qt.Horizontal)
        speed_slider.setMinimum(0)
        speed_slider.setMaximum(10)
        speed_slider.setValue(1)
        speed_slider.setTickInterval(1)
        speed_slider.valueChanged.connect(self.update_speed)
        speed_layout.addWidget(speed_slider)
        layout.addLayout(speed_layout)

        # Allow tilt checkbox
        allow_tilt_layout = QHBoxLayout()
        allow_tilt_label = QLabel("Allow tilt")
        allow_tilt_checkbox = QCheckBox()
        allow_tilt_checkbox.setChecked(False)
        allow_tilt_checkbox.stateChanged.connect(self.allow_tilt)
        allow_tilt_layout.addWidget(allow_tilt_label)
        allow_tilt_layout.addWidget(allow_tilt_checkbox)
        allow_tilt_layout.addStretch(1)
        layout.addLayout(allow_tilt_layout)

        layout.addSpacing(10)

        # Image stream recorder
        self.ros2_bag_image_stream_recorder = Ros2BagAPI([f'/{robot_name}/camera/image_color' for robot_name in self.robot_names] +
                                                         [f'/{robot_name}/ground_truth_pose' for robot_name in self.robot_names])
        image_stream_bag_name = str(Path.home()) + "/Desktop/image_stream1"
        self.ros2_bag_image_stream_recorder.set_bag_file_name(
            image_stream_bag_name)

        image_stream_bag_file_name_layout = QHBoxLayout()
        image_stream_bag_file_name_label = QLabel(
            "Image Stream Bag File Name:", self)
        image_stream_bag_file_name_layout.addWidget(
            image_stream_bag_file_name_label)
        self.image_stream_bag_file_name_input = QLineEdit(self)
        self.image_stream_bag_file_name_input.setText(image_stream_bag_name)
        self.image_stream_bag_file_name_input.textChanged.connect(
            self.ros2_bag_image_stream_recorder.set_bag_file_name)
        image_stream_bag_file_name_layout.addWidget(
            self.image_stream_bag_file_name_input)
        layout.addLayout(image_stream_bag_file_name_layout)

        record_image_stream_layout = QHBoxLayout()
        record_image_stream_label = QLabel("Record image stream", self)
        record_image_stream_layout.addWidget(record_image_stream_label)
        start_recording_image_stream_button = QPushButton("Start")
        start_recording_image_stream_button.clicked.connect(
            self.ros2_bag_image_stream_recorder.start_recording)
        record_image_stream_layout.addWidget(
            start_recording_image_stream_button)
        stop_recording_image_stream_button = QPushButton("Stop")
        stop_recording_image_stream_button.clicked.connect(
            self.ros2_bag_image_stream_recorder.stop_recording)
        record_image_stream_layout.addWidget(
            stop_recording_image_stream_button)
        layout.addLayout(record_image_stream_layout)

        layout.addSpacing(16)

        # Ros2bag playback
        self.ros2_bag_playback_apis = []
        for i, robot_name in enumerate(self.robot_names):
            ros2_bag_playback = Ros2BagAPI()
            ros2_bag_playback.set_bag_file_name(
                "/home/joshuabird/Desktop/MH_01")
            ros2_bag_playback.set_playback_remapping_old_topic(
                "cam0/image_raw")
            ros2_bag_playback.set_playback_remapping_new_topic(
                f'{robot_name}/camera/image_color')
            self.ros2_bag_playback_apis.append(ros2_bag_playback)

            playback_file_name_layout = QHBoxLayout()
            playback_file_name_label = QLabel(
                "Playback Bag File Name:", self)
            playback_file_name_layout.addWidget(
                playback_file_name_label)
            self.playback_file_name_input = QLineEdit(self)
            self.playback_file_name_input.setText(
                "/home/joshuabird/Desktop/MH_01")
            self.playback_file_name_input.textChanged.connect(
                self.ros2_bag_playback_apis[i].set_bag_file_name)
            playback_file_name_layout.addWidget(
                self.playback_file_name_input)
            layout.addLayout(playback_file_name_layout)

            playback_image_stream_layout = QHBoxLayout()
            playback_image_stream_label = QLabel("Playback image stream", self)
            playback_image_stream_layout.addWidget(playback_image_stream_label)
            start_playback_image_stream_button = QPushButton("Start")
            start_playback_image_stream_button.clicked.connect(
                self.ros2_bag_playback_apis[i].start_playback)
            playback_image_stream_layout.addWidget(
                start_playback_image_stream_button)
            stop_playback_image_stream_button = QPushButton("Stop")
            stop_playback_image_stream_button.clicked.connect(
                self.ros2_bag_playback_apis[i].stop_playback)
            playback_image_stream_layout.addWidget(
                stop_playback_image_stream_button)
            layout.addLayout(playback_image_stream_layout)

            playback_topic_remap_layout = QHBoxLayout()
            playback_topic_remap_label = QLabel(
                "Topic Remap (Old --> New)", self)
            playback_topic_remap_label.setSizePolicy(
                QSizePolicy.Expanding, QSizePolicy.Expanding)
            playback_topic_remap_layout.addWidget(playback_topic_remap_label)
            playback_old_topic_input = QLineEdit(self)
            playback_old_topic_input.setText(f'cam0/image_raw')
            playback_old_topic_input.textChanged.connect(
                self.ros2_bag_playback_apis[i].set_playback_remapping_old_topic)
            playback_topic_remap_layout.addWidget(
                playback_old_topic_input)
            playback_new_topic_input = QLineEdit(self)
            playback_new_topic_input.setText(
                f'{robot_name}/camera/image_color')
            playback_new_topic_input.textChanged.connect(
                self.ros2_bag_playback_apis[i].set_playback_remapping_old_topic)
            playback_topic_remap_layout.addWidget(
                playback_new_topic_input)
            layout.addLayout(playback_topic_remap_layout)

            playback_rate_layout = QHBoxLayout()
            playback_rate_label = QLabel("Playback rate (0-100%)")
            playback_rate_layout.addWidget(playback_rate_label)
            playback_rate_slider = QSlider(Qt.Horizontal)
            playback_rate_slider.setMinimum(0)
            playback_rate_slider.setMaximum(10)
            playback_rate_slider.setValue(10)
            playback_rate_slider.setTickInterval(1)
            playback_rate_slider.valueChanged.connect(
                self.ros2_bag_playback_apis[i].set_playback_rate)
            playback_rate_layout.addWidget(playback_rate_slider)
            layout.addLayout(playback_rate_layout)

            layout.addSpacing(16)

        # Trajectory recorder
        self.ros2_bag_trajectory_recorder = Ros2BagAPI([f'/{robot_name}/ground_truth_pose' for robot_name in self.robot_names] +
                                                       [f'/{robot_name}/camera_pose' for robot_name in self.robot_names] +
                                                       [f'/{robot_name}/successfully_merged' for robot_name in self.robot_names] +
                                                       ["/sim3_transform"])
        trajectory_bag_name = str(Path.home()) + "/Desktop/trajectory"
        self.ros2_bag_trajectory_recorder.set_bag_file_name(
            trajectory_bag_name)

        trajectory_bag_file_name_layout = QHBoxLayout()
        trajectory_bag_file_name_label = QLabel(
            "Trajectory Bag File Name:", self)
        trajectory_bag_file_name_layout.addWidget(
            trajectory_bag_file_name_label)
        self.trajectory_bag_file_name_input = QLineEdit(self)
        self.trajectory_bag_file_name_input.setText(trajectory_bag_name)
        self.trajectory_bag_file_name_input.textChanged.connect(
            self.ros2_bag_trajectory_recorder.set_bag_file_name)
        trajectory_bag_file_name_layout.addWidget(
            self.trajectory_bag_file_name_input)
        layout.addLayout(trajectory_bag_file_name_layout)

        record_trajectory_layout = QHBoxLayout()
        record_trajectory_label = QLabel("Record trajectory", self)
        record_trajectory_layout.addWidget(record_trajectory_label)
        start_record_trajectory_button = QPushButton("Start")
        start_record_trajectory_button.clicked.connect(
            self.ros2_bag_trajectory_recorder.start_recording)
        record_trajectory_layout.addWidget(
            start_record_trajectory_button)
        stop_record_trajectory_button = QPushButton("Stop")
        stop_record_trajectory_button.clicked.connect(
            self.ros2_bag_trajectory_recorder.stop_recording)
        record_trajectory_layout.addWidget(
            stop_record_trajectory_button)
        layout.addLayout(record_trajectory_layout)

        layout.addStretch(1)

        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)
        self.show()

        # create timer for spinning
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timer_update)
        self.timer.start(10)
        print("Started timer")

    def timer_update(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        self.timer.start(10)

    def update_speed(self, value):
        self.speed = value
        for robot_viewers in self.robot_viewers:
            robot_viewers.updateSpeed(self.speed)
        self.speed_label.setText("Speed: {}".format(self.speed))

    def allow_tilt(self, state):
        if state == Qt.Checked:
            for robot_viewers in self.robot_viewers:
                robot_viewers.setAllowTilt(True)
        else:
            for robot_viewers in self.robot_viewers:
                robot_viewers.setAllowTilt(False)

    def shutdown(self):
        self.node.destroy_node()
        self.ros2_bag_image_stream_recorder.shutdown()
        self.ros2_bag_trajectory_recorder.shutdown()
        for ros2_bag_playback in self.ros2_bag_playback_apis:
            ros2_bag_playback.shutdown()


def sigint_handler(sig, frame, window):
    sys.stderr.write('\r')
    QApplication.quit()
    window.shutdown()
    rclpy.shutdown()

    sys.exit(0)


def main(args=None):
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    signal.signal(signal.SIGINT, partial(sigint_handler, window=window))
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
