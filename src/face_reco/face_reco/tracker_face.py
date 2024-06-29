import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import pigpio

class FaceTracker(Node):
    def __init__(self):
        super().__init__('tracker_face')
        self.face_sub = self.create_subscription(Point, '/detected_face', self.face_callback, 1)

        # Initialisation pigpio to control servos
        self.pi = pigpio.pi()
        self.servo_pin_tilt = 17  
        self.servo_pin_pan = 18   

        # Initial configuration
        self.tilt_angle = 90
        self.pan_angle = 90
        self.set_servo_angle(self.servo_pin_tilt, self.tilt_angle)
        self.set_servo_angle(self.servo_pin_pan, self.pan_angle)

        # Dimensions de l'image
        self.image_width = 640
        self.image_height = 480

        # Filtre pour lisser les commandes des servos
        self.filter_factor = 0.2  # Facteur de lissage (0-1, plus proche de 1 signifie moins de lissage)

    def face_callback(self, msg):

        # Coordonnées du centre du visage
        face_x = msg.x
        face_y = msg.y

        # Coordonnées du centre de l'image
        image_center_x = self.image_width / 2
        image_center_y = self.image_height / 2

        # Calcul de l'erreur de position
        error_x = face_x - image_center_x
        error_y = face_y - image_center_y

        # Ajustement des angles des servo-moteurs
        k_p = 0.02  # Réduire le coefficient proportionnel pour des mouvements plus doux
        self.pan_angle -= k_p * error_x
        self.tilt_angle += k_p * error_y  # Inverser la direction du tilt

        # Contrainte des angles entre 0 et 180 degrés
        self.pan_angle = max(0, min(180, self.pan_angle))
        self.tilt_angle = max(0, min(180, self.tilt_angle))

        # Appliquer le filtre pour lisser les commandes des servos
        self.pan_angle = self.filter_factor * self.pan_angle + (1 - self.filter_factor) * self.pan_angle
        self.tilt_angle = self.filter_factor * self.tilt_angle + (1 - self.filter_factor) * self.tilt_angle

        # Mise à jour des angles des servo-moteurs
        self.set_servo_angle(self.servo_pin_tilt, self.tilt_angle)
        self.set_servo_angle(self.servo_pin_pan, self.pan_angle)

        self.get_logger().info(f"Pan angle: {self.pan_angle}, Tilt angle: {self.tilt_angle}")

    def set_servo_angle(self, servo, angle):
        pulsewidth = 500 + (angle * 2000 / 180)  # Convertir l'angle en largeur d'impulsion (500-2500 microsecondes)
        self.pi.set_servo_pulsewidth(servo, pulsewidth)

    def destroy_node(self):
        # Arrêter les servos avant de quitter
        self.pi.set_servo_pulsewidth(self.servo_pin_tilt, 0)
        self.pi.set_servo_pulsewidth(self.servo_pin_pan, 0)
        self.pi.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    face_tracker = FaceTracker()
    try:
        rclpy.spin(face_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        face_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
