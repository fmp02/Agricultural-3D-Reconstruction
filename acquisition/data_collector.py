#!/usr/bin/env python3

""" Nodo ROS per la raccolta di un dataset RGB-D da un robot LIMO che si muove nella scena """
import os
import json
import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge

# Costruisce la matrice di trasformazione
def pose_matrix(x, y, yaw):

    c = math.cos(yaw)
    s = math.sin(yaw)

    T = np.eye(4)

    T[0, 0] = c
    T[0, 1] = -s
    T[1, 0] = s
    T[1, 1] = c

    T[0, 3] = x
    T[1, 3] = y

    return T

""" NODO ROS """

class ReconstructionCollector(Node):

    def __init__(self):

        super().__init__("reconstruction_data_collector")

        # Parametri 
        
        self.goal_x = -1.975    # target da raggiungere sulla scena
        self.goal_y = -0.025

        self.forward_speed = 0.4    # velocità
        self.turn_gain = 0.8        # guadagno rotazione

        self.save_every_n = 3       # un frame salvato ogni 3

        # dir in cui viene salvato il dataset finale che contiene:
            # 1- dir color (da rgb sensor)
            # 2- dir depth (da depth sensor)
            # 3- dir poses
            # 4- file json parametri camera
        self.output_dir = os.path.expanduser("~/reconstruction_data")

        self.color_dir = os.path.join(self.output_dir, "color")
        self.depth_dir = os.path.join(self.output_dir, "depth")
        self.pose_dir = os.path.join(self.output_dir, "poses")

        os.makedirs(self.color_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)
        os.makedirs(self.pose_dir, exist_ok=True)

        # parametri camera
        self.fx = 221.7
        self.fy = 221.7
        self.cx = 128.0
        self.cy = 128.0

        self.bridge = CvBridge()

        # stati
        self.x = None
        self.y = None
        self.yaw = None

        self.latest_rgb = None
        self.latest_depth = None

        self.frame_counter = 0
        self.saved_frames = 0

        self.aligning = True
        self.finished = False

        # publishers e subscribers
        self.create_subscription(Image, "/camera/color/image_raw", self.rgb_callback, 10)
        self.create_subscription(Image, "/camera/depth/image_raw", self.depth_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # funzione cotrollo richiamata ogni 0.05 s
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.save_intrinsics()
        
    """ CALLBACKS """
    
    # Callback chiamata per acquisire l'ultima immagine RGB disponibile
    def rgb_callback(self, msg):

        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    # Callback chiamata per acquisire l'ultima depth disponibile
    def depth_callback(self, msg):

        if msg.encoding == "32FC1":
            depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")

        else:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough").astype(np.float32)

        self.latest_depth = depth

    # Callback per aggiornare la posa del robot 
    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.yaw = math.atan2(siny, cosy)

    """ CONTROL-LOOP """

        #1. Calcola distanza e orientamento rispetto al target (pallino rosso)
        #2. Allinea il robot verso la destinazione
        #3. Avanza
        #4. Salva i dati
    def control_loop(self):

        if self.finished:
            self.publish_cmd(0.0, 0.0)
            return

        if self.x is None:
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        distance = math.hypot(dx, dy)

        if distance < 0.10:
            self.publish_cmd(0.0, 0.0)
            self.finished = True
            return

        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - self.yaw)

        if self.aligning:

            if abs(yaw_error) < 0.05:
                self.aligning = False

            else:
                self.publish_cmd(0.0, self.turn_gain * yaw_error)
                return

        omega = 0.5 * yaw_error

        self.publish_cmd(self.forward_speed, omega)
        self.save_frame()

    # Funzione chiamata per salvare i dati acquisiti nelle dir corrispondenti
    def save_frame(self):

        if self.latest_rgb is None:
            return

        if self.latest_depth is None:
            return

        self.frame_counter += 1

        if self.frame_counter % self.save_every_n != 0:
            return

        name = f"frame_{self.saved_frames:05d}"

        rgb_path = os.path.join(self.color_dir, f"{name}.png")
        depth_path = os.path.join(self.depth_dir, f"{name}.npy")
        pose_path = os.path.join(self.pose_dir, f"{name}.txt")

        cv2.imwrite(rgb_path, self.latest_rgb)

        np.save(depth_path, self.latest_depth.astype(np.float32))

        T = pose_matrix(
            self.x,
            self.y,
            self.yaw
        )

        np.savetxt(pose_path, T, fmt="%.8f")

        self.saved_frames += 1
        
    """ HELPERS """

    # Salva i parametri della camera in formato JSON.
    def save_intrinsics(self):

        data = {
            "width": 256,
            "height": 256,
            "fx": self.fx,
            "fy": self.fy,
            "cx": self.cx,
            "cy": self.cy
        }

        with open(os.path.join(self.output_dir, "camera_intrinsics.json"),"w") as f:
            json.dump(data, f, indent=2)

    # Riporta angolo nell'intervallo [-pi, pi].
    def normalize_angle(self, a):

        return (
            (a + math.pi)
            % (2 * math.pi)
            - math.pi
        )

    # Pubblica il messaggio di velocità sul topic
    def publish_cmd(self, linear, angular):

        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        self.cmd_pub.publish(msg)

""" MAIN """

def main():

    rclpy.init()
    node = ReconstructionCollector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()