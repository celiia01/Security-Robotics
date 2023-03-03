#!/usr/bin/python3.8

import rospy
from geometry_msgs.msg import TwistStamped


class VelPub:
	def __init__(self):
		self.pub = rospy.Publisher('desired_vel', TwistStamped, queue_size=10)

	def publish(self):
		msg = TwistStamped()
		msg.twist.linear.x = 0.5
		msg.twist.angular.z = 1
		msg.header.stamp = rospy.Time.now()
		self.pub.publish(msg)
		

def main():
	rospy.init_node("vel_publisher", anonymous=True)

	vel_node = VelPub()
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		vel_node.publish()
		rate.sleep()

if __name__ == "__main__":
    main()
