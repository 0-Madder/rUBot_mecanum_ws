#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from tensorflow.keras.models import load_model
import time
from datetime import datetime
import os
import threading


class ImageClassifierNode:
    def __init__(self):
        rospy.loginfo("Inicializando nodo de clasificación de imágenes...")

        script_dir = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(script_dir, "../models/keras_model.h5")
        labels_path = os.path.join(script_dir, "../models/labels.txt")

        self.model = load_model(model_path)
        self.labels = self.load_labels(labels_path)
        self.input_shape = self.model.input_shape[1:3]

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1)
        self.class_pub = rospy.Publisher("/predicted_class", String, queue_size=1)
        rospy.Subscriber("/capture_toggle", Bool, self.toggle_callback)

        self.capture_enabled = True
        self.capture_dir = os.path.expanduser("~/rUBot_mecanum_ws/src/rubot_projects/rUBot_captures")
        os.makedirs(self.capture_dir, exist_ok=True)
        self.create_class_dirs()
        self.last_capture_time = time.time()
        self.capture_interval = 1.0

        self.image_count = 0
        self.debug_interval = 10

        rospy.loginfo("Nodo de clasificación de imágenes listo. Esperando imágenes...")

    def load_labels(self, path):
        with open(path, 'r') as f:
            return [line.strip().split(' ', 1)[1] for line in f.readlines()]

    def create_class_dirs(self):
        for class_name in self.labels:
            os.makedirs(os.path.join(self.capture_dir, class_name), exist_ok=True)

    def toggle_callback(self, msg):
        self.capture_enabled = msg.data
        state = "ACTIVADA" if self.capture_enabled else "DESACTIVADA"
        rospy.loginfo(f"[Clasificador] Captura automática {state}")

    def image_callback(self, msg):
        self.image_count += 1
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            resized = cv2.resize(cv_image, self.input_shape)
            img = resized.astype(np.float32) / 255.0
            img = np.expand_dims(img, axis=0)

            predictions = self.model.predict(img)
            class_index = np.argmax(predictions)
            class_name = self.labels[class_index]
            confidence = predictions[0][class_index]

            if self.image_count % self.debug_interval == 0:
                rospy.loginfo(f"[Clasificador] Imagen #{self.image_count} - Clase: {class_name} ({confidence:.2f})")

            self.class_pub.publish(class_name)

        except CvBridgeError as e:
            rospy.logerr(f"[Clasificador] Error de CvBridge: {e}")
        except Exception as e:
            rospy.logerr(f"[Clasificador] Error procesando imagen: {e}")


class SignalBehaviorNode:
    def __init__(self):
        rospy.loginfo("Inicializando nodo de comportamiento reactivo...")

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.signal_detected = "Nothing"
        self.signal_position = None
        self.signal_time = rospy.Time.now()

        self.detection_distance_threshold = 0.4  # metros
        self.detection_timeout = rospy.Duration(5.0)  # temps durant el qual és vàlida una senyal

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/predicted_class", String, self.class_callback)

        self.command_lock = threading.Lock()
        self.control_thread = threading.Thread(target=self.motion_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

        rospy.loginfo("Nodo de comportamiento reactivo listo.")

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def class_callback(self, msg):
        with self.command_lock:
            class_name = msg.data

            # Registrar posición ficticia de la señal 0.5m delante
            self.signal_detected = class_name
            self.signal_time = rospy.Time.now()
            self.signal_position = (self.robot_x + 0.5, self.robot_y)

            rospy.loginfo(f"[Comportamiento] Señal detectada: {class_name} en ({self.signal_position[0]:.2f}, {self.signal_position[1]:.2f})")

    def motion_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            twist = Twist()
            with self.command_lock:
                cmd = self.signal_detected
                sig_time = self.signal_time #moment en el que es detecta la senyal
                sig_pos = self.signal_position #(x,y) posició del robot quan detecta la senyal

            if rospy.Time.now() - sig_time > self.detection_timeout: #Si el temps que ha passat desde que s'ha detectat l'última senyal  es major que el maxim temps de processarment s'una sneyal, continuem amb NOthing (linia recta) fins que en detectem una altra.
                # Expirar señal
                cmd = "Nothing"
                sig_pos = None

            if sig_pos is not None:
                distance = np.sqrt((self.robot_x - sig_pos[0])**2 + (self.robot_y - sig_pos[1])**2)
                rospy.loginfo_throttle(2.0, f"[Comportamiento] Distancia a señal '{cmd}': {distance:.2f} m")

                if distance <= self.detection_distance_threshold:
                    if cmd in ["Stop", "Give_Way"]:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        rospy.loginfo_throttle(2.0, f"[Comportamiento] PARAR por señal '{cmd}'")
                    elif cmd == "Turn_Left":
                        twist.linear.x = 0.0
                        twist.angular.z = 0.5
                        rospy.loginfo_throttle(2.0, "[Comportamiento] GIRAR IZQUIERDA")
                    elif cmd == "Turn_Right":
                        twist.linear.x = 0.0
                        twist.angular.z = -0.5
                        rospy.loginfo_throttle(2.0, "[Comportamiento] GIRAR DERECHA")
                    else:
                        twist.linear.x = 0.2
                        twist.angular.z = 0.0
                else:
                    # Aún no estamos suficientemente cerca
                    twist.linear.x = 0.2
                    twist.angular.z = 0.0
                    rospy.loginfo_throttle(2.0, "[Comportamiento] AVANZAR hacia la señal...")
            else:
                # Nada detectado o señal expirada
                twist.linear.x = 0.2
                twist.angular.z = 0.0
                rospy.loginfo_throttle(2.0, "[Comportamiento] AVANZAR (Nada detectado)")

            self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("keras_takePictures_detect_signs_move2", anonymous=True)

    classifier_node = ImageClassifierNode()
    behavior_node = SignalBehaviorNode()

    rospy.loginfo("Sistema completo de clasificación y comportamiento iniciado.")
    rospy.spin()