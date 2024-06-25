#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2

class FaceDetector(Node):
    def __init__(self):
        super().__init__('detect_face')
        self.image_sub = self.create_subscription(Image, '/image_in', self.callback, rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.face_pub = self.create_publisher(Point, '/detected_face', 1)
        self.image_pub = self.create_publisher(Image, '/image_out', 1)
        self.bridge = CvBridge()
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    
    def callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.05, 8, minSize=(120, 120))
        
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 5)
            self.publish_coordinates(x, y, w, h)

        # Publish image_out
        image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(image_message)
        
    def publish_coordinates(self, x, y, w, h):
        point = Point()
        point.x = x + w / 2  # Coordinate x center 
        point.y = y + h / 2  # Coordinate y center
        point.z = 0  # Z not used
        self.face_pub.publish(point)
        self.get_logger().info(f"Published coordinates: x={point.x}, y={point.y}")

def main(args=None):
    #Initialisation communication ROS
    rclpy.init(args=args)

    face_detector = FaceDetector()
    rclpy.spin(face_detector)
    face_detector.destroy_node()

    #Continue le node
    #rclpy.spin(node)

    #Stop communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
