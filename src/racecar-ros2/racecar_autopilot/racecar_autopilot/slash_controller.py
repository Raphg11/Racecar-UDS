#!/usr/bin/env python
import rclpy
import math
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class SlashController(Node):

    def __init__(self):
        super().__init__("controller")

        # Init subscribers
        self.sub_ref = self.create_subscription(Twist, "ctl_ref", self.read_ref, 1)
        self.sub_prop = self.create_subscription(
            Float32MultiArray, "prop_sensors", self.read_arduino, 1
        )
        self.sub_laser = self.create_subscription(
            Twist, "car_position", self.read_laser, 1
        )

        # Init publishers
        self.pub_cmd = self.create_publisher(Twist, "prop_cmd", 1)

        # Timer
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.timed_controller)

        # Parameters

        # Controller
        # Reglages modifiables EN DIRECT, sans rebuild, ex. :
        #   ros2 param set /controller y_sign -1.0
        #   ros2 param set /controller theta_sign -1.0
        #   ros2 param set /controller steering_offset 0.02
        self.declare_parameter("steering_offset", 0.0)  # To adjust according to the vehicle
        # Signes des mesures LiDAR (mettre -1.0 si inverse)
        self.declare_parameter("y_sign", 1.0)      # y > 0 : auto a GAUCHE du centre du corridor
        self.declare_parameter("theta_sign", -1.0)  # INVERSE par defaut : LiDAR monte a 180 deg (valide sur l'auto)
        self.steering_offset = 0.0
        self.y_sign = 1.0
        self.theta_sign = -1.0
        self.max_steering = 0.52  # [rad] ~30 deg, saturation du servo sur l'Arduino

        # TODO: test on real racecar to validate
        # Design offline, paste here. Shapes must match the x you assemble below.
        self.params_autopilot = {
            "K": np.array([0.316, 0.538]),
            "ubar": 0.0
        }
        self.params_parking = {
            "K": np.array([[1,0,0],
                           [0,0.375,0.6]])
        }

        # Memory

        # References Inputs
        self.propulsion_ref = 0
        self.steering_ref = 0
        self.high_level_mode = 0  # Control mode of this controller node

        # Ouput commands
        self.propulsion_cmd = 0  # Command sent to propulsion
        self.arduino_mode = 0  # Control mode
        self.steering_cmd = 0  # Command sent to the steering servo

        # Sensings inputs
        self.laser_y = 0
        self.laser_theta = 0
        self.velocity = 0
        self.position = 0

        # Filters
        self.laser_y_old = 0
        self.laser_dy_fill = 0

    #######################################
    def timed_controller(self):

        # Lecture des parametres (modifiables en direct avec ros2 param set)
        self.steering_offset = self.get_parameter("steering_offset").value
        self.y_sign = self.get_parameter("y_sign").value
        self.theta_sign = self.get_parameter("theta_sign").value

        # Computation of dy / dt with filtering
        self.laser_dy_fill = (
            0.9 * self.laser_dy_fill + 0.1 * (self.laser_y - self.laser_y_old) / self.dt
        )
        self.laser_y_old = self.laser_y

        if self.high_level_mode < 0:
            # Full stop mode
            self.propulsion_cmd = 0  # Command sent to propulsion
            self.arduino_mode = 0  # Control mode
            self.steering_cmd = 0  # Command sent to the steering servo

        else:

            # APP2 (open-loop steering) Controllers Bellow
            if self.high_level_mode == 1:
                # Open-Loop propulsion and steering
                self.propulsion_cmd = self.propulsion_ref
                self.arduino_mode = 1
                self.steering_cmd = self.steering_ref + self.steering_offset

            # For compatibility mode 0 needs to be closed-loop velocity
            elif self.high_level_mode == 0:
                # Closed-loop velocity on arduino, open-loop steering
                self.propulsion_cmd = self.propulsion_ref
                self.arduino_mode = 2
                self.steering_cmd = self.steering_ref + self.steering_offset

            elif self.high_level_mode == 2:
                # Closed-loop position on arduino, open-loop steering
                self.propulsion_cmd = self.propulsion_ref
                self.arduino_mode = 3
                self.steering_cmd = self.steering_ref + self.steering_offset

            # ----------------------------------------------------------
            # TODO: test on real racecar to validate
            # APP4 — catalog of signals available
            #
            # 1. Measurements (last value received)
            #    self.position       encoder, longitudinal position [m]
            #    self.velocity       encoder, longitudinal speed [m/s]
            #    self.laser_y        lidar, lateral position [m]
            #    self.laser_theta    lidar, heading [rad]
            #    self.laser_dy_fill  filtered dy/dt [m/s]
            #
            # 2. References (from ctl_ref)
            #    self.propulsion_ref  longitudinal ref (speed or position,
            #                         depends on the high-level joystick mode)
            #    self.steering_ref    steering / lateral ref
            #
            # 3. Controllable actions — two channels; meaning depends on
            #    arduino_mode
            #    u[0]  propulsion
            #          arduino_mode == 1 → open-loop voltage V [V]
            #          arduino_mode == 2 → closed-loop velocity setpoint
            #          arduino_mode == 3 → closed-loop position setpoint
            #    u[1]  steering δ  (steering_offset is added after the policy)
            #    self.arduino_mode  is a choice you must set below (1 / 2 / 3)
            # ----------------------------------------------------------

            elif self.high_level_mode == 3:
                # Autopilot (high-level mode 3)

                #########################################################
                # TODO: complètez — assemblage de x, r et arduino_mode

                # x = np.array([[self.velocity],
                #               [self.laser_y],
                #               [self.laser_theta]])   # pick from the catalog

                # r = np.array([
                #     [2.0],
                #     [0.0],
                #     [0.0]
                # ])

                # u = self.ctl_autopilot(x, r)

                # self.propulsion_cmd = u[0]
                # self.steering_cmd = u[1] + self.steering_offset
                # self.arduino_mode = 0  # complètez: 1, 2 ou 3 (0 = sortie nulle)
                
                # Autopilot (Option #2)

                # 1. États Lidar uniquement
                x1 = np.array([
                    self.y_sign * self.laser_y,
                    self.theta_sign * self.laser_theta
                ])
                r1 = np.array([
                    self.steering_ref,  # yd (consigne latérale)
                    0.0                 # theta_d = 0 (parallèle au mur)
                ])

                # 2. Calcul du braquage delta_desire (K1 * x1)
                delta_desire = float(np.clip(self.ctl_autopilot(x1, r1),
                                             -self.max_steering, self.max_steering))

                # 3. Trajet direct de vd vers l'Arduino (ligne pointillée)
                self.propulsion_cmd = self.propulsion_ref  # vd (ex: 2.0 m/s)
                self.steering_cmd = delta_desire + self.steering_offset

                # 4. Mode 5 = cruise control APP4 sur l'Arduino (vel_ubar + vel_k2 * erreur)
                self.arduino_mode = 5

                # Debug (2x par seconde) pour valider les signes
                self.get_logger().info(
                    f"[autopilot] y={x1[0]:+.3f} m  theta={x1[1]:+.3f} rad  "
                    f"delta={delta_desire:+.3f} rad  v={self.velocity:.2f} m/s",
                    throttle_duration_sec=0.5,
                )
                #########################################################

            elif self.high_level_mode == 4:
                # Parking (high-level mode 4)

                #########################################################
                # TODO: complètez — assemblage de x, r et arduino_mode

                # LQR complet : x = [position encodeur, y, theta]
                x = np.array([[self.position],
                              [self.y_sign * self.laser_y],
                              [self.theta_sign * self.laser_theta]])

                r = np.array([[self.propulsion_ref],   # position cible [m]
                              [self.steering_ref],     # position laterale cible [m]
                              [0.0]])                  # parallele aux murs

                u = self.ctl_parking(x, r)  # u = -K (x - r)

                # u[0] = consigne de vitesse [m/s] (limitee), u[1] = braquage [rad]
                self.propulsion_cmd = float(np.clip(u[0, 0], -1.0, 1.0))
                self.steering_cmd = float(np.clip(u[1, 0], -self.max_steering,
                                                  self.max_steering)) + self.steering_offset
                self.arduino_mode = 2  # PI de vitesse sur l'Arduino (retombe a 0 a la cible)

                # Debug (2x par seconde)
                self.get_logger().info(
                    f"[parking] pos={self.position:+.3f}/{self.propulsion_ref:+.2f} m  "
                    f"y={x[1, 0]:+.3f} m  theta={x[2, 0]:+.3f} rad  "
                    f"v_cmd={self.propulsion_cmd:+.2f} m/s  delta={self.steering_cmd:+.3f} rad",
                    throttle_duration_sec=0.5,
                )
                #########################################################

            elif self.high_level_mode == 5:
                # Template for custom controllers
                self.steering_cmd = 0 + self.steering_offset
                self.propulsion_cmd = 0
                self.arduino_mode = 0  # Mode ??? on arduino

            elif self.high_level_mode == 6:
                # Reset encoders
                self.propulsion_cmd = 0
                self.arduino_mode = 4
                self.steering_cmd = 0

            elif self.high_level_mode == 7:
                # Closed-loop velocity open-loop steering
                self.propulsion_cmd = self.propulsion_ref
                self.arduino_mode = 2
                self.steering_cmd = self.steering_ref + self.steering_offset

            elif self.high_level_mode == 8:
                # Template for custom controllers

                self.steering_cmd = 0 + self.steering_offset
                self.propulsion_cmd = 0
                self.arduino_mode = 1  # Mode ??? on arduino

            elif self.high_level_mode == 9:
                # Template for custom controllers

                self.steering_cmd = 0 + self.steering_offset
                self.propulsion_cmd = 0
                self.arduino_mode = 1  # Mode ??? on arduino

        self.send_arduino()

    #######################################
    def ctl_autopilot(self, x, r, t=0, params=None):

        params = self.params_autopilot if params is None else params

        K = params["K"]
        ubar = params["ubar"]
        u = ubar - np.dot(K, (x - r))

        return u

    #######################################
    def ctl_parking(self, x, r, t=0, params=None):

        params = self.params_parking if params is None else params

        K = params["K"]
        u = - K @ (x - r)

        return u

    #######################################
    def read_ref(self, ref_msg):

        # Read received references
        self.propulsion_ref = ref_msg.linear.x
        self.high_level_mode = ref_msg.linear.z
        self.steering_ref = ref_msg.angular.z

    #######################################
    def read_laser(self, msg):

        self.laser_y = msg.linear.y
        self.laser_theta = msg.angular.z

    #######################################
    def read_arduino(self, msg):

        # Read feedback from arduino
        self.velocity = msg.data[1]
        self.position = msg.data[0]

    ##########################################################################################
    def send_arduino(self):

        # Init encd_info msg
        cmd_prop = Twist()

        # Msg
        cmd_prop.linear.x = float(self.propulsion_cmd)  # Command sent to propulsion
        cmd_prop.linear.z = float(self.arduino_mode)  # Control mode

        cmd_prop.angular.z = float(
            self.steering_cmd
        )  # Command sent to the steering servo

        # Publish cmd msg
        self.pub_cmd.publish(cmd_prop)


def main(args=None):
    rclpy.init(args=args)
    node = SlashController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


#########################################

if __name__ == "__main__":
    main()
