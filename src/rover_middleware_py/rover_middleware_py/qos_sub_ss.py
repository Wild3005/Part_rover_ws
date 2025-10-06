import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from datetime import datetime

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')
        self.bridge = CvBridge()
        self.image_counter = 0
        
        # Print current working directory
        current_dir = os.getcwd()
        print(f"🗂️ Current working directory: {current_dir}")
        
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.sub = self.create_subscription(
            CompressedImage,                 # <--- bedanya di sini
            '/image_screenshot/compressed',  # pastikan sesuai topic
            self.callback,
            qos_profile
        )
        
        print(f"🎯 Subscribed to topic: /image_screenshot/compressed")
        print("🔄 Waiting for messages...")

    def callback(self, msg):
        print("📬 Received message from topic!")
        
        # Decode CompressedImage
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
        
        print(f"Frame shape: {frame.shape if frame is not None else 'None'}")
        
        if frame is not None:
            # Buat nama file dengan timestamp dan path absolut
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"screenshot_{timestamp}_{self.image_counter:03d}.jpg"
            
            # Dapatkan direktori saat ini
            current_dir = os.getcwd()
            full_path = os.path.join(current_dir, filename)
            
            print(f"💾 Trying to save to: {full_path}")
            
            # Simpan sebagai JPEG dengan parameter kualitas
            success = cv2.imwrite(full_path, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            
            if success:
                self.image_counter += 1
                file_size = os.path.getsize(full_path) if os.path.exists(full_path) else 0
                print(f"✅ Screenshot berhasil disimpan: {filename} ({file_size} bytes)")
                print(f"📁 Lokasi: {full_path}")
            else:
                print(f"❌ Gagal menyimpan screenshot: {filename}")
            
            # Tampilkan preview
            # cv2.imshow("Screenshot", frame)
            # cv2.waitKey(1)
        else:
            print("⚠️ Frame kosong atau gagal di-decode!")

def main():
    rclpy.init()
    node = CompressedImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
