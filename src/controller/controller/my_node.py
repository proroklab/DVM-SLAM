import sys
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QHBoxLayout, QWidget
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPixmap, QImage, QCursor
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import signal
from functools import partial


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

    def image_callback(self, msg):
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
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

        self.twist.angular.y = offset.y()/100
        self.twist.angular.z = -offset.x()/100

        self.cmd_vel_publisher.publish(self.twist)

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_W:
            self.twist.linear.x = 1.0
        elif key == Qt.Key_A:
            self.twist.linear.y = 1.0
        elif key == Qt.Key_S:
            self.twist.linear.x = -1.0
        elif key == Qt.Key_D:
            self.twist.linear.y = -1.0

        self.cmd_vel_publisher.publish(self.twist)

    def keyReleaseEvent(self, event):
        key = event.key()
        print(key)
        if key == Qt.Key_W or key == Qt.Key_S:
            self.twist.linear.x = 0.0
        elif key == Qt.Key_A or key == Qt.Key_D:
            self.twist.linear.y = 0.0

        self.cmd_vel_publisher.publish(self.twist)


class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        # ROS2 init
        rclpy.init(args=None)
        self.node = Node('image_subscriber_node')
        # spin once, timeout_sec 5[s]
        timeout_sec_rclpy = 5
        rclpy.spin_once(self.node, timeout_sec=timeout_sec_rclpy)
        print("Connected to ros")

        self.setWindowTitle('Image Viewer')
        central_widget = QWidget()

        layout = QHBoxLayout()
        robot1_viewer = ImageViewer("robot1", self.node)
        layout.addWidget(robot1_viewer)
        robot2_viewer = ImageViewer("robot2", self.node)
        layout.addWidget(robot2_viewer)

        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)
        self.show()

        # create timer for spining
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timer_update)
        self.timer.start(10)
        print("Started timer")

    def timer_update(self):
        rclpy.spin_once(self.node)
        self.timer.start(10)


def sigint_handler(sig, frame, node):
    sys.stderr.write('\r')
    QApplication.quit()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


def main(args=None):
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    signal.signal(signal.SIGINT, partial(sigint_handler, node=window.node))
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
