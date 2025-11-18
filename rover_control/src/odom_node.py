#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
import tf2_ros
import geometry_msgs.msg

NUM_WHEELS = 6

class OdomNode:
    def __init__(self):
        self.ticks_per_rev = rospy.get_param("~ticks_per_rev", 1024)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.1)
        self.track_width = rospy.get_param("~track_width", 0.4)

        self.last_ticks = None
        self.last_time = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odom_broadcaster = tf2_ros.TransformBroadcaster()

        self.sub = rospy.Subscriber("encoder_ticks", Int32MultiArray, self.enc_cb)

    def enc_cb(self, msg):
        now = rospy.Time.now()
        if self.last_ticks is None:
            self.last_ticks = list(msg.data)
            self.last_time = now
            return

        dt = (now - self.last_time).to_sec()
        if dt <= 0.0:
            return
        self.last_time = now

        dticks = [msg.data[i] - self.last_ticks[i] for i in range(NUM_WHEELS)]
        self.last_ticks = list(msg.data)

        # Left wheels: 0,1,2 ; Right wheels: 3,4,5
        left_ticks = sum(dticks[0:3]) / 3.0
        right_ticks = sum(dticks[3:6]) / 3.0

        left_dist = (left_ticks / self.ticks_per_rev) * 2.0 * math.pi * self.wheel_radius
        right_dist = (right_ticks / self.ticks_per_rev) * 2.0 * math.pi * self.wheel_radius

        d = (left_dist + right_dist) / 2.0
        dtheta = (right_dist - left_dist) / self.track_width

        self.yaw += dtheta
        self.x += d * math.cos(self.yaw)
        self.y += d * math.sin(self.yaw)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        vx = d / dt
        vth = dtheta / dt
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.odom_broadcaster.sendTransform(t)

if __name__ == "__main__":
    rospy.initNode("odom_node")
    node = OdomNode()
    rospy.spin()
