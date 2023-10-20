import sys
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from PyQt5.QtGui import QPixmap, QImage
import signal
from functools import partial


class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        self.setWindowTitle('Image Viewer')
        central_widget = QWidget()
        layout = QVBoxLayout()
        self.label = QLabel()
        layout.addWidget(self.label)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)
        self.show()

        # ROS2 init
        rclpy.init(args=None)
        self.node = Node('image_subscriber_node')
        self.subscription = self.node.create_subscription(
            Image, 'robot1/camera/image_color', self.image_callback, 10)
        # spin once, timeout_sec 5[s]
        timeout_sec_rclpy = 5
        rclpy.spin_once(self.node, timeout_sec=timeout_sec_rclpy)
        print("Connected to ros")

        # create timer for spining
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timer_update)
        self.timer.start(10)
        print("Started timer")

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
        # Display the image using PyQt\
        pixmap = QPixmap.fromImage(image)
        self.label.setPixmap(pixmap)

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
