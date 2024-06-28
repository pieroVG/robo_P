import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class FaceDetector(Node):
    def __init__(self):
        super().__init__('face_detector')
        self.image_sub = self.create_subscription(CompressedImage,'/image_raw/compressed',self.callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        
        self.face_pub = self.create_publisher(Point, '/detected_face', 10)
        self.image_pub = self.create_publisher(CompressedImage, '/image_out/compressed', 10)
        self.bridge = CvBridge()

        haarcascade_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
        self.face_cascade = cv2.CascadeClassifier(haarcascade_path)

    def callback(self, data):
        np_arr = np.frombuffer(data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.05, 8, minSize=(120, 120))
        
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 5)
            self.publish_coordinates(x, y, w, h)

        # Convert in ROS message
        compressed_image_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        self.image_pub.publish(compressed_image_msg)

    def publish_coordinates(self, x, y, w, h):
        point = Point()
        point.x = x + w / 2  
        point.y = y + h / 2  
        point.z = 0.00
        self.face_pub.publish(point)
        self.get_logger().info(f"Face coordinates: x={point.x}, y={point.y}")

def main(args=None):
    rclpy.init(args=args)
    face_detector = FaceDetector()
    rclpy.spin(face_detector)
    face_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
