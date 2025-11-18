#!/usr/bin/env python3
import rospy
import math
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray

NUM_WHEELS = 6

class AckermannKinematics:
    def __init__(self):
        # Geometry
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.1)   # m
        self.wheel_base = rospy.get_param("~wheel_base", 0.5)       # front-rear axle distance
        self.track_width = rospy.get_param("~track_width", 0.4)     # left-right distance
        self.mid_axle_factor = rospy.get_param("~mid_axle_factor", 1.0)  # scaling for middle wheels

        # Publishers
        self.pub_speed_cmd = rospy.Publisher("wheel_speed_cmd", Float32MultiArray, queue_size=1)
        self.pub_steer_cmd = rospy.Publisher("steering_cmd", Float32MultiArray, queue_size=1)

        # Subscribe to high-level command
        self.sub_cmd = rospy.Subscriber("ackermann_cmd", AckermannDriveStamped,
                                        self.cmd_cb, queue_size=1)

    def cmd_cb(self, msg):
        v = msg.drive.speed                # m/s
        delta = msg.drive.steering_angle   # rad, front steering

        # 4WS: front axle +delta, rear axle -delta
        delta_front = delta
        delta_rear = -delta

        # Protect small steering angle
        if abs(delta) < 1e-3:
            # Straight driving
            w = v / self.wheel_radius
            speeds = [w, w, w, w, w, w]
            # steering: 0 for all
            steering = [0.0, 0.0, 0.0, 0.0]
        else:
            # Turning radius based on front steering
            R = self.wheel_base / math.tan(delta_front)
            # Angular velocity of vehicle
            omega = v / R

            # Radii for wheels, approximate:
            R_FL = math.sqrt((R - self.track_width/2.0)**2 + (self.wheel_base/2.0)**2)
            R_FR = math.sqrt((R + self.track_width/2.0)**2 + (self.wheel_base/2.0)**2)
            R_RL = R_FL
            R_RR = R_FR
            R_ML = R - self.track_width/2.0
            R_MR = R + self.track_width/2.0

            # Linear speeds
            v_FL = omega * R_FL
            v_FR = omega * R_FR
            v_RL = omega * R_RL
            v_RR = omega * R_RR
            v_ML = omega * R_ML * self.mid_axle_factor
            v_MR = omega * R_MR * self.mid_axle_factor

            # Angular wheel speeds
            w_FL = v_FL / self.wheel_radius
            w_FR = v_FR / self.wheel_radius
            w_ML = v_ML / self.wheel_radius
            w_MR = v_MR / self.wheel_radius
            w_RL = v_RL / self.wheel_radius
            w_RR = v_RR / self.wheel_radius

            speeds = [w_FL, w_ML, w_RL, w_FR, w_MR, w_RR]

            # Steering angles for wheels (FL, FR, RL, RR)
            steer_FL = math.atan2(self.wheel_base/2.0, R - self.track_width/2.0)
            steer_FR = math.atan2(self.wheel_base/2.0, R + self.track_width/2.0)
            steer_RL = -steer_FL
            steer_RR = -steer_FR
            steering = [steer_FL, steer_FR, steer_RL, steer_RR]

        speed_msg = Float32MultiArray(data=speeds)
        steer_msg = Float32MultiArray(data=steering)
        self.pub_speed_cmd.publish(speed_msg)
        self.pub_steer_cmd.publish(steer_msg)

if __name__ == "__main__":
    rospy.initNode("ackermann_kinematics_node")
    node = AckermannKinematics()
    rospy.spin()
