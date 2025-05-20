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


class ImageClassifierNode:
    def __init__(self):
        rospy.loginfo("Inicializando nodo de clasificación de imágenes...")

        # Ruta de modelo y etiquetas
        script_dir = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(script_dir, "../models/keras_model.h5")
        labels_path = os.path.join(script_dir, "../models/labels.txt")

        # Cargar modelo
        self.model = load_model(model_path)
        self.labels = self.load_labels(labels_path)
        self.input_shape = self.model.input_shape[1:3]

        # Subscripciones y publicaciones
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1)
        self.class_pub = rospy.Publisher("/predicted_class", String, queue_size=1)
        rospy.Subscriber("/capture_toggle", Bool, self.toggle_callback)

        # Captura de imágenes
        self.capture_enabled = True
        self.capture_dir = os.path.expanduser("~/rUBot_mecanum_ws/src/rubot_projects/rUBot_captures")
        os.makedirs(self.capture_dir, exist_ok=True)
        self.create_class_dirs()
        self.last_capture_time = time.time()
        self.capture_interval = 1.0  # segundos

        # Contador para debug visual
        self.image_count = 0
        self.debug_interval = 10  # mostrar logs de predicción cada 10 imágenes

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
            rospy.logdebug("[Clasificador] Imagen recibida.")
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            resized = cv2.resize(cv_image, self.input_shape)
            img = resized.astype(np.float32) / 255.0
            img = np.expand_dims(img, axis=0)

            predictions = self.model.predict(img)
            class_index = np.argmax(predictions)
            class_name = self.labels[class_index]
            confidence = predictions[0][class_index]

            # Mostrar log de clasificación cada X imágenes
            if self.image_count % self.debug_interval == 0:
                rospy.loginfo(f"[Clasificador] Imagen #{self.image_count} - Clase: {class_name} ({confidence:.2f})")

            self.class_pub.publish(class_name)

            #if self.capture_enabled:
                #current_time = time.time()
                #if current_time - self.last_capture_time >= self.capture_interval:
                    #timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    #filename = f"{class_name}_{timestamp}.jpg"
                    #filepath = os.path.join(self.capture_dir, class_name, filename)
                    #cv2.imwrite(filepath, cv_image)
                    #rospy.loginfo(f"[Clasificador] Imagen guardada: {filepath}")
                    #self.last_capture_time = current_time

        except CvBridgeError as e:
            rospy.logerr(f"[Clasificador] Error de CvBridge: {e}")
        except Exception as e:
            rospy.logerr(f"[Clasificador] Error procesando imagen: {e}")


class SignalBehaviorNode:
    def __init__(self):
        rospy.loginfo("Inicializando nodo de comportamiento reactivo...")

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.signal_x = None
        self.signal_y = None
        self.last_detection_time = None

        self.detection_timeout = 10.0  # segundos
        self.detection_distance_threshold = 0.4  # metros

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/predicted_class", String, self.class_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        rospy.loginfo("Nodo de comportamiento reactivo listo.")

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def class_callback(self, msg):
        class_name = msg.data
        self.signal_x = self.robot_x + 0.5  # posición ficticia 0.5m delante
        self.signal_y = self.robot_y
        self.last_detection_time = rospy.Time.now()

        time_diff = rospy.Time.now() - self.last_detection_time
        if time_diff.to_sec() < self.detection_timeout:
            distance = np.sqrt((self.robot_x - self.signal_x)**2 + (self.robot_y - self.signal_y)**2)
            rospy.loginfo(f"[Comportamiento] Distancia a '{class_name}': {distance:.2f} m")

            twist = Twist()
            if distance <= self.detection_distance_threshold:
                if class_name in ["Stop", "Give_Way"]:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif class_name == "Turn_Left":
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5
                elif class_name == "Turn_Right":
                    twist.linear.x = 0.0
                    twist.angular.z = -0.5
                self.cmd_vel_pub.publish(twist)


if __name__ == "__main__":
    rospy.init_node("keras_takePictures_detect_signs_move2", anonymous=True)
    
    # Lanzar los dos nodos dentro del mismo script
    classifier_node = ImageClassifierNode()
    behavior_node = SignalBehaviorNode()

    rospy.loginfo("Sistema completo de clasificación y comportamiento iniciado.")
    rospy.spin()
