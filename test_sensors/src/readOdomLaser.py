#!/usr/bin/python3.8

import rospy
import message_filters
import numpy as np
import math
import matplotlib.pyplot as plt

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Twist
from multiprocessing import Lock


class TestNode:
	def __init__(self):
		self.lidar_sub = message_filters.Subscriber('base_scan', LaserScan)
		self.teleop_sub = rospy.Subscriber('teleop_vel', Twist, self.callback_teleop, queue_size=10)
		self.odom_sub = message_filters.Subscriber('odom', Odometry)
		self.desired_vel_sub = message_filters.Subscriber('desired_vel', TwistStamped)
		self.ts = message_filters.TimeSynchronizer([self.lidar_sub, self.odom_sub, self.desired_vel_sub], 10)
		self.ts.registerCallback(self.callback)
		self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.teleop_pub = rospy.Publisher('desired_vel', TwistStamped, queue_size=10)
		self.vel_msg = Twist()
		self.correct_direction = 0
		self.lock = Lock()
		self.trayectoria_top = False
		self.trayectoria_bottom = False
	
	def callback_teleop(self, msg: Twist):
		vel_stamped = TwistStamped()
		vel_stamped.header.stamp = rospy.Time.now()
		vel_stamped.twist = msg
		self.teleop_pub.publish(vel_stamped)

	def send_vel(self, v, w):
		self.vel_msg.linear.x = v
		self.vel_msg.angular.z = w																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																					
		self.vel_pub.publish(self.vel_msg)

	def skipObstacle(self, scan_ranges, middle, top, bottom, desired_v, desired_w):
		if(np.amin(middle) < 1):
			self.lock.acquire()																																																																																																																																																																																																																																																																																																																																																																																																																																																				
			if(np.where(scan_ranges == np.amin(scan_ranges))[0][0] < 600):
				v = 0
				w = math.pi
				self.correct_direction += w
				print("correct < 600: ",self.correct_direction)
			else:
				v = 0
				w = -math.pi
				self.correct_direction += w
				print("ANorher: ",self.correct_direction)
			self.lock.release()
		elif(np.amin(top) < 1 or np.amin(bottom) < 1):
			print("LO veo")
			v = desired_v
			w = 0
		elif(self.correct_direction != 0):
			v = 0
			self.lock.acquire()
			if (self.correct_direction < 0):
				w = math.pi
				self.correct_direction += math.pi 
			else:
				w = -math.pi
				self.correct_direction -= math.pi
			self.lock.release()
		else:
			v = desired_v
			w = desired_w

		return v,w


	def callback(self, scan: LaserScan, odom: Odometry, desired_vel: TwistStamped):
		x = odom.pose.pose.position.x
		y = odom.pose.pose.position.y
		desired_v = desired_vel.twist.linear.x																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																							
		desired_w = desired_vel.twist.angular.z
		scan_ranges = np.array(scan.ranges)
		middle = np.array(scan_ranges[478:641])
		top = np.array(scan_ranges[642:])
		bottom = np.array(scan_ranges[:477])
		#print(middle)

		#print(len(scan.ranges), desired_v)																																																						
		self.lock.acquire()
		# NO HA INICIADO NINGUNA TRAYECTORIA PARA ESQUIVAR
		if(np.amin(middle) < 0.5 and not(self.trayectoria_bottom) and not(self.trayectoria_top)):	
			print("DETECTO SCANER CENTRAL: ", np.where(middle == np.amin(middle))[0][0] + 478, " metros: ", np.amin(middle))
			v = 0																																																																																																																																																																																																																																																																																																																																																																																																																																																			
			if(np.where(middle == np.amin(middle))[0][0] < 82):
				w = math.pi
				self.correct_direction += w
				self.trayectoria_bottom = True
			else:
				w = -math.pi
				self.correct_direction += w
				self.trayectoria_top = True
			print("giro Acumulado: ",self.correct_direction)
		#HA INICIADO UNA TRAYECTORIA PARA ESQUIVAR
		elif(np.amin(middle) < 0.5 and (self.trayectoria_bottom or self.trayectoria_top) ):
			v = 0
			w = math.pi if (self.trayectoria_bottom) else -math.pi
			self.correct_direction += w
		elif(np.amin(top) < 0.5 or np.amin(bottom) < 0.5):
			print("minimo de top: ", np.amin(top),  "o bottom: ", np.amin(bottom))
			v = desired_v
			w = 0
		elif(abs(self.correct_direction) > 0.0):
			print("Mimimo mayor que 0.5: ", np.amin(scan_ranges), " Corrijo el giro")
			v = 0
			if (self.correct_direction < 0):
				w = math.pi
				self.correct_direction += math.pi 
			else:
				w = -math.pi
				self.correct_direction -= math.pi

			if (abs(self.correct_direction) > 0 and abs(self.correct_direction) < 0.00001):
				self.correct_direction = 0

			print("Giro restante: ", self.correct_direction)
		else:
			v = desired_v
			w = desired_w
		self.lock.release()
		
		self.send_vel(v,w)
		#rospy.loginfo(rospy.get_caller_id() + "Min range %f", np.minimum(scan_ranges))

def main():
    rospy.init_node("test_node", anonymous=True)

    test_node = TestNode()

    rospy.spin() 

if __name__ == "__main__":
    main()
