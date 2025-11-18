#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray, Int16MultiArray, Int32MultiArray
from dynamic_reconfigure.server import Server
from rover_control.cfg import WheelPIDConfig
import math
import time

NUM_WHEELS = 6

class WheelPID:
    def __init__(self):
        self.ticks_per_rev = rospy.get_param("~ticks_per_rev", 1024)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.1)  # m

        self.kp = 0.5
        self.ki = 0.0
        self.kd = 0.0
        self.max_pwm = 200

        self.desired_vel = [0.0] * NUM_WHEELS     # rad/s
        self.last_ticks = [0] * NUM_WHEELS
        self.last_time = None
        self.integral = [0.0] * NUM_WHEELS
        self.prev_error = [0.0] * NUM_WHEELS

        self.srv = Server(WheelPIDConfig, self.reconfig_cb)
        self.pub_pwm = rospy.Publisher("wheel_pwm_cmd", Int16MultiArray, queue_size=1)
        self.sub_cmd = rospy.Subscriber("wheel_speed_cmd", Float32MultiArray,
                                        self.cmd_cb, queue_size=1)
        self.sub_enc = rospy.Subscriber("encoder_ticks", Int32MultiArray,
                                        self.enc_cb, queue_size=1)

    def reconfig_cb(self, config, level):
        self.kp = config.kp
        self.ki = config.ki
        self.kd = config.kd
        self.max_pwm = config.max_pwm
        return config

    def cmd_cb(self, msg):
        for i in range(min(NUM_WHEELS, len(msg.data))):
            self.desired_vel[i] = msg.data[i]

    def enc_cb(self, msg):
        now = rospy.Time.now().to_sec()
        if self.last_time is None:
            self.last_time = now
            self.last_ticks = list(msg.data)
            return

        dt = now - self.last_time
        if dt <= 0.0:
            return
        self.last_time = now

        wheel_pwm = Int16MultiArray()
        wheel_pwm.data = [0] * NUM_WHEELS

        for i in range(NUM_WHEELS):
            dticks = msg.data[i] - self.last_ticks[i]
            self.last_ticks[i] = msg.data[i]

            # rad/s = (dticks / ticks_per_rev) * 2*pi / dt
            vel = (dticks / float(self.ticks_per_rev)) * 2.0 * math.pi / dt
            error = self.desired_vel[i] - vel

            self.integral[i] += error * dt
            derivative = (error - self.prev_error[i]) / dt
            self.prev_error[i] = error

            u = self.kp * error + self.ki * self.integral[i] + self.kd * derivative
            pwm = int(max(-self.max_pwm, min(self.max_pwm, u * self.max_pwm)))  # simple scale

            wheel_pwm.data[i] = pwm

        self.pub_pwm.publish(wheel_pwm)

if __name__ == "__main__":
    rospy.initNode("wheel_pid_node")
    node = WheelPID()
    rospy.spin()
