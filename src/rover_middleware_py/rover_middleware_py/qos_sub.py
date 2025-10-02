import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')
        self.bridge = CvBridge()
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.sub = self.create_subscription(
            CompressedImage,                 # <--- bedanya di sini
            'topic_best_effort/compressed',  # pastikan sesuai topic
            self.callback,
            qos_profile
        )

    def callback(self, msg):
        # Cara decode CompressedImage
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # bisa IMREAD_GRAYSCALE
        cv2.imshow("Compressed Image", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = CompressedImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
