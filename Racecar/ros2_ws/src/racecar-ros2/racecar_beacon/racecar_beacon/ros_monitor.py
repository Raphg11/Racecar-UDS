#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import socket
import threading
from struct import pack

from racecar_beacon.utils import yaw_from_quaternion


class ROSMonitor(Node):
    def __init__(self):
        super().__init__("ros_monitor")

        # Robot state
        self.id = int(0xFFFF)
        self.position = tuple([float(0), float(0), float(0)])
        self.obstacle_detected = bool(False)

        # Socket parameters
        self.host = self.declare_parameter("host", "127.0.0.1").value
        self.remote_request_port = self.declare_parameter(
            "remote_request_port", 65432
        ).value
        self.broadcast = self.declare_parameter("broadcast", "127.0.0.255").value
        self.position_broad_port = self.declare_parameter(
            "pos_broadcast_port", 65431
        ).value

        self.remote_request_t = threading.Thread(target=self.remote_request_loop)


        
        #----------------- Raph (goat) ----------------------#
        # Abonnement à l'odométrie et Lidar (code du labo)
        self.subscribe_Odometry = self.create_subscription(Odometry, "/odometry/filtered", self.odometry_callback, 1)
        self.subscribe_Lidar = self.create_subscription(LaserScan, "/scan", self.scan_callback, 1)

        # Timer pour le service PositionBroadcast (1Hz)
        self.broadcast_timer = self.create_timer(1.0, self.position_broadcast_callback)
        self.remote_request_t.start()

        self.get_logger().info(f"{self.get_name()} started.")



    def remote_request_loop(self):
        # NOTE: It is recommended to initialize your socket here.
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((self.host, self.remote_request_port))
        server_socket.listen()
        server_socket.settimeout(1.0)
        self.get_logger().info(f"Serveur TCP en écoute sur {self.host}:{self.remote_request_port}")
        
        while rclpy.ok():
            try:
                
                connexion, addresse = server_socket.accept()
                
                with connexion:
                   
                    data = connexion.recv(4)
                    if not data:
                        continue
                        
                    commande = data.decode('ascii')
                    reponse = b''
                    
                    if commande == "RPOS":
                        reponse = pack("fff4x", self.position[0], self.position[1], self.position[2])
                        
                    elif commande == "OBSF":
                        val_obstacle = 1 if self.obstacle_detected else 0
                        reponse = pack("I12x", val_obstacle)
                        
                    elif commande == "RBID":
                        reponse = pack("I12x", self.id)
                        
                    else:
                        reponse = pack("16x")
                        
                    connexion.sendall(reponse)
                    
            except socket.timeout:
                
                pass
            except Exception as e:
                self.get_logger().error(f"Erreur de connexion TCP : {e}")

        server_socket.close()


    def shutdown(self):
        """Gracefully shutdown the threads BEFORE terminating the node."""
        self.remote_request_t.join()


    def odometry_callback(self, msg: Odometry) -> None:
           x = msg.pose.pose.position.x
           y = msg.pose.pose.position.y
           orientation = msg.pose.pose.orientation
           theta = yaw_from_quaternion(orientation)
           
           
           print(f"Position -> X: {x:.3f}, Y: {y:.3f}, Theta: {theta:.3f}")
           self.position = (x,y,theta)

    def scan_callback(self, msg: LaserScan) -> None:
        #un obstacle à moins de 1 mètre
        self.obstacle_detected = False
        for distance in msg.ranges:
            if 0.0 <distance < 1.0:
                self.obstacle_detected = True
                break


    def position_broadcast_callback(self):
        """Envoie la position et l'ID du robot en UDP broadcast toutes les secondes."""
        try:
            
            socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
            socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            
            payload = pack("fffI", self.position[0], self.position[1], self.position[2], self.id)
            
            socket.sendto(payload, (self.broadcast, self.position_broad_port))
            socket.close()
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors du broadcast UDP : {e}")
        


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ROSMonitor()
        rclpy.get_default_context().on_shutdown(node.shutdown)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
