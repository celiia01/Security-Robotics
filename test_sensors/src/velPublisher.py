#!/usr/bin/python3.8

import rospy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class VelPub:
	def __init__(self):
		self.pub = rospy.Publisher('desired_vel', TwistStamped, queue_size=1)
		self.robot_sub = rospy.Subscriber('/robot_0/odom', Odometry, self.robot_callback, queue_size=1)
		self.goal_sub = rospy.Subscriber('/robot_1/odom', Odometry, self.goal_callback, queue_size=1)
		self.robot_x = 0.0
		self.robot_y = 0.0
		self.robot_theta = 0.0
		self.goal_x = 0.0
		self.goal_y = 0.0
	
	def robot_callback(self, data):
		self.robot_x = data.pose.pose.position.x
		self.robot_y = data.pose.pose.position.y
		orientation_q = data.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(_, _, self.robot_theta) = euler_from_quaternion (orientation_list)
	
	def goal_callback(self, data):
		self.goal_x = data.pose.pose.position.x
		self.goal_y = data.pose.pose.position.y

	def publish(self, event=None):
		print("Robot pose-> x: ", self.robot_x, ", y: ", self.robot_y, ", theta: ", self.robot_theta)
		print("Goal-> x: ", self.goal_x, ", y: ", self.goal_y)
		msg = TwistStamped()
		msg.twist.linear.x = 0.5
		msg.twist.angular.z = 0.0
		msg.header.stamp = rospy.Time.now()
		self.pub.publish(msg)
		

def main():
	rospy.init_node("vel_publisher", anonymous=True)

	vel_node = VelPub()
	rospy.Timer(rospy.Duration(1.0/10.0), vel_node.publish)

	rospy.spin()

if __name__ == "__main__":
    main()
