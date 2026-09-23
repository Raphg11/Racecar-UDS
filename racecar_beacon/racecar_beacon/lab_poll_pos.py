#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf_transformations import euler_from_quaternion

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

from racecar_beacon.utils import yaw_from_quaternion


class PositionPoller(Node):
    def __init__(self):
        super().__init__("position_poller")
        # TODO: Add your subscription(s) here. Use the following syntax:
        #   `self.subscription = self.create_subscription(<type>, <topic>, <self.callback>, 1)`
        self.subscription = self.create_subscription(
            Odometry, 
            "/odometry/filtered", 
            self.odometry_callback, 
            1
        )
        self.get_logger().info(f"{self.get_name()} started.")

    def odometry_callback(self, msg: Odometry) -> None:
        # Récupération de la position X et Y
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Récupération et conversion de l'orientation (Quaternion -> Theta)
        orientation = msg.pose.pose.orientation
        theta = yaw_from_quaternion(orientation)
        
        # Affichage à l'écran demandé par l'exercice
        print(f"Position -> X: {x:.3f}, Y: {y:.3f}, Theta: {theta:.3f}")


def main(args=None):
    try:
        rclpy.init(args=args)
        node = PositionPoller()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()
