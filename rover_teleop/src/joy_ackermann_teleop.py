#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

class JoyAckermannTeleop:
    def __init__(self):
        self.max_speed = rospy.get_param("~max_speed", 1.0)
        self.max_steering = rospy.get_param("~max_steering", 0.6)

        self.axis_speed = rospy.get_param("~axis_speed", 1)      # usually left stick vertical
        self.axis_steer = rospy.get_param("~axis_steer", 0)      # left stick horizontal
        self.deadband = rospy.get_param("~deadband", 0.05)

        self.pub = rospy.Publisher("ackermann_cmd", AckermannDriveStamped, queue_size=1)
        self.sub = rospy.Subscriber("joy", Joy, self.joy_cb)

    def joy_cb(self, msg):
        try:
            speed_axis = msg.axes[self.axis_speed]
            steer_axis = msg.axes[self.axis_steer]
        except IndexError:
            return

        if abs(speed_axis) < self.deadband:
            speed_axis = 0.0
        if abs(steer_axis) < self.deadband:
            steer_axis = 0.0

        speed = self.max_speed * speed_axis
        steering = self.max_steering * steer_axis

        cmd = AckermannDriveStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.drive.speed = speed
        cmd.drive.steering_angle = steering
        self.pub.publish(cmd)

if __name__ == "__main__":
    rospy.initNode("joy_ackermann_teleop")
    node = JoyAckermannTeleop()
    rospy.spin()
