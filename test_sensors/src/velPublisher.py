#!/usr/bin/python3.8

import rospy
import time
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from auxiliar import *


class VelPub:
	WCHANGE = 15 * math.pi / 180.0
	WBEF = np.inf
	TRAYECTORIA_POSITIVA = False
	XROBOTBEF = 0
	YROBOTBEF = 0
	WBEFORE = 15 * math.pi / 180.0
	ESTANCADO = False
	CONTADOR = 10
	TIME = time.clock_gettime(time.CLOCK_REALTIME)
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
		# print("Robot pose-> x: ", self.robot_x, ", y: ", self.robot_y, ", theta: ", self.robot_theta)
		# print("Goal-> x: ", self.goal_x, ", y: ", self.goal_y)
		t = time.clock_gettime(time.CLOCK_REALTIME)
		msg = TwistStamped()
		vFixed = 0.7
		wXR = np.array([self.robot_x, self.robot_y, self.robot_theta])
		wTR = hom(wXR)

		wXG = np.array([self.goal_x, self.goal_y, 0])
		wTG = hom(wXG)

		rTG = np.linalg.inv(wTR) @ wTG
		rXG = loc(rTG)
		# print(rXG)
		if abs(rXG[0]) > 0.15 or abs(rXG[1]) > 0.15:
			
			radius = (rXG[0]**2 + rXG[1]**2) / (2 * rXG[1])
			if abs(radius) <= vFixed/self.WCHANGE:
				w = vFixed / radius
				#print("CHANGE: ", radius)
			else:
				w = -self.WCHANGE if radius < 0 else self.WCHANGE

			if (t - self.TIME) > 0.2:
				if self.robot_x == self.XROBOTBEF and self.robot_y == self.YROBOTBEF:
					w = 0
					self.ESTANCADO = True
				else:
					self.ESTANCADO = False
				self.TIME = t
				
			# print(self.WBEFORE, w, self.CHANGE)
			# if abs(self.WBEFORE) == abs(w) and self.WBEFORE != w and not self.CHANGE:
			# 	print("ENTOR")
			# 	w = 0

			if abs(rXG[1]) < 0.1 and abs(rXG[2]) < 0.1:
				w = 0
			if abs(rXG[0]) < 0.1 and abs(rXG[2]) < 0.1:
				vFixed = 0
				w = math.pi if rXG[1] > 0 else -math.pi
				#print("GIRA")
			if w != 0 and ( abs(vFixed / w - rXG[1]) < 0.1  or abs(vFixed / w - rXG[0]) < 0.1)and abs(w) == math.pi:
				vFixed -= 0.1
				#print(".1")
			
			if self.ESTANCADO:
				w = 0
		
			msg.twist.linear.x = vFixed
			msg.twist.angular.z = round(w, 3)

			#print("Plann v: ", vFixed, " w: ", w)

			self.RXGBEF = rXG
			self.XROBOTBEF = self.robot_x
			self.YROBOTBEF = self.robot_y
		else:
			msg.twist.linear.x = 0
			msg.twist.angular.z = 0
		
		msg.header.stamp = rospy.Time.now()
		self.pub.publish(msg)
		# print("publisher: ", time.clock_gettime(time.CLOCK_REALTIME) - t)
		
		# timeAct = time.clock_gettime(time.CLOCK_REALTIME)
		# while (time.clock_gettime(time.CLOCK_REALTIME) - timeAct < 1):
		# 	msg.header.stamp = rospy.Time.now()
		# 	self.pub.publish(msg)
		
		

def main():
	rospy.init_node("vel_publisher", anonymous=True)

	vel_node = VelPub()
	rospy.Timer(rospy.Duration(1.0/10.0), vel_node.publish)

	rospy.spin()

if __name__ == "__main__":
    main()
