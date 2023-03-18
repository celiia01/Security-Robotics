#!/usr/bin/python3.8

import rospy
import message_filters
import numpy as np
import math
import matplotlib.pyplot as plt
import time

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
		self.timeCall = 0.1
	
	def callback_teleop(self, msg: Twist):
		vel_stamped = TwistStamped()
		vel_stamped.header.stamp = rospy.Time.now()
		vel_stamped.twist = msg
		self.teleop_pub.publish(vel_stamped)

	def send_vel(self, v, w):
		self.vel_msg.linear.x = v
		self.vel_msg.angular.z = w																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																					
		self.vel_pub.publish(self.vel_msg)

	def skipObstacle(self, scan_ranges, middle, top, bottom, desired_v, desired_w, distance):
		self.lock.acquire()
		# NO HA INICIADO NINGUNA TRAYECTORIA PARA ESQUIVAR
		if(np.amin(middle) < distance and not(self.trayectoria_bottom) and not(self.trayectoria_top)):	
			print("DETECTO SCANER CENTRAL: ", np.where(middle == np.amin(middle))[0][0], " metros: ", np.amin(middle))
			v = 0	
			# 61 es el punto central del vector																																																																																																																																																																																																																																																																																																																																																																																																																																																		
			if(np.where(middle == np.amin(middle))[0][0] < len(middle)/2):
				w = math.pi
				self.correct_direction += w
				self.trayectoria_bottom = True
			else:
				w = -math.pi
				self.correct_direction += w
				self.trayectoria_top = True
			print("giro Acumulado: ",self.correct_direction)
		#HA INICIADO UNA TRAYECTORIA PARA ESQUIVAR
		elif(np.amin(middle) < distance and (self.trayectoria_bottom or self.trayectoria_top) ):
			v = 0
			w = math.pi if (self.trayectoria_bottom) else -math.pi
			self.correct_direction += w
		elif(np.amin(top) < distance or np.amin(bottom) < distance):
			self.trayectoria_bottom = False
			self.trayectoria_top = False
			print("minimo de top: ", np.amin(top),  "o bottom: ", np.amin(bottom))
			v = desired_v
			w = 0
		elif(abs(self.correct_direction) > 0.0):
			print("Mimimo mayor que 0.5: ", np.amin(scan_ranges), " Corrijo el giro")
			v = 0
			w = math.pi if (self.correct_direction < 0) else -math.pi
			self.correct_direction += w

			if (abs(self.correct_direction) > 0 and abs(self.correct_direction) < 0.00001):
				self.correct_direction = 0

			print("Giro restante: ", self.correct_direction)
		else:
			v = desired_v
			w = desired_w

		self.lock.release()

		return v,w
	
	# Minimun Distance from the robot to the most proximate obstacle
	def dist(self, v: float, w: float, 
	  middle: np.array, top: np.array, bottom: np.array):
		self.lock.acquire()
		dt = self.timeCall
		self.lock.release()
		if v > 0 and v < 0.0001:
			distance = 30.0
			return distance
		elif w > 0 and w < 0.0001:
			distance = np.amin(middle)
			return distance - v * dt 
		elif w > 0.0001:
			distance = np.amin(middle) if np.amin(middle) < np.amin(bottom) else np.amin(bottom)
		else:
			distance = np.amin(middle) if np.amin(middle) < np.amin(top) else np.amin(top)

		return distance - (v/w * (w * dt))
	
	# OPTIMIZAR BUCLE!!!!
	def velocities(self, middle: np.array, top: np.array, bottom: np.array, 
		vAct: float, wAct: float, vMin: float, vMax: float, wMin: float, wMax: float):
		aceleMaxV = 0.7
		aceleMaxW = math.pi

		Vr = []
		Vs = []
		t1 = time.clock_gettime(time.CLOCK_REALTIME)
		for v in np.arange(vMin, vMax, 0.01):
			for w in np.arange(wMin, wMax, math.pi/180.0):
			
				distance = TestNode.dist(self, v, w, middle,top,bottom)

				limitVLow = vAct - aceleMaxV * self.timeCall
				limitVUp = vAct + aceleMaxV * self.timeCall
				limitWLow = wAct - aceleMaxW * self.timeCall
				limitWUp = wAct + aceleMaxW * self.timeCall
				limitV = math.sqrt(2.0 * distance * aceleMaxV)
				limitW = math.sqrt(2.0 * distance * aceleMaxW)


				if (v >= limitVLow and v <= limitVUp and w >= limitWLow and w <= limitWUp and v <= limitV and w <= limitW):
					Vr.append([v,w])

				Vs.append([v,w])				
				# Accelerations for breakage??

		t2 = time.clock_gettime(time.CLOCK_REALTIME) - t1
		print(t2)



		return Vs, Vr
		
	# Admissible Velocities
	def velocityAdmissible(self, middle: np.array, top: np.array, bottom: np.array, 
			vA):
		
		Va = []

		for i in vA:
			for v in i:	
				#print(v)
				distance = TestNode.dist(self, v[0],v[1],middle,top,bottom)
				# Accelerations for breakage??
				aceleMaxV = 0.7
				aceleMaxW = math.pi

				limitV = math.sqrt(2.0 * distance * aceleMaxV)
				limitW = math.sqrt(2.0 * distance * aceleMaxW)

				if (v[0] <= limitV and v[1] <= limitW):
					Va.append([v[0],v[1]])
		
		return Va
	
	def velocityDynamic(self, vAct: float, wAct: float, 
		     vMin: float, vMax: float, wMin: float, wMax: float ):
		# Specifications of the robot
		aceleMaxV = 0.7
		aceleMaxW = math.pi

		Vd = []

		for v in np.arange(vMin, vMax, 0.01):
			for w in np.arange(wMin, wMax, math.pi/180.0):
			

				limitVLow = vAct - aceleMaxV * self.timeCall
				limitVUp = vAct + aceleMaxV * self.timeCall
				limitWLow = wAct - aceleMaxW * self.timeCall
				limitWUp = wAct + aceleMaxW * self.timeCall

				if (v >= limitVLow and v <= limitVUp and w >= limitWLow and w <= limitWUp):
					Vd.append([v,w])

		return Vd
		
		

	def callback(self, scan: LaserScan, odom: Odometry, desired_vel: TwistStamped):
		x = odom.pose.pose.position.x
		y = odom.pose.pose.position.y
		# Actual Velocities
		vAct = odom.twist.twist.linear.x
		wAct = odom.twist.twist.angular.z
		# Desired Velocities
		desired_v = desired_vel.twist.linear.x																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																							
		desired_w = desired_vel.twist.angular.z
		distance = 0.5
		scan_ranges = np.array(scan.ranges)
		middle = np.array(scan_ranges[478:600])
		top = np.array(scan_ranges[601:])
		bottom = np.array(scan_ranges[:477])

		# ¿Pongo las velocidades máximas o las deseadas?
		vMin = 0.0
		wMin = -math.pi
		vMax = 0.7
		wMax = math.pi

		
		x = np.arange(vMin, vMax, 0.01)
		y = np.arange(wMin, wMax, math.pi/180.0)

		Vs = np.array([[0 for w in np.arange(wMin, wMax, math.pi/180.0)] for v in np.arange(vMin, vMax, 0.01)])

		plt.matshow(Vs)
		plt.show()

		_, Vr = TestNode.velocities(self, middle, top, bottom, vAct, wAct, vMin, vMax, wMin, wMax)
		
		

		
		
		#print(middle)

		#print(len(scan.ranges), desired_v)																																																						
		v = desired_v
		w = desired_w
		
		
		self.send_vel(v,w)
		#rospy.loginfo(rospy.get_caller_id() + "Min range %f", np.minimum(scan_ranges))

def main():
    rospy.init_node("test_node", anonymous=True)

    test_node = TestNode()

    rospy.spin() 

if __name__ == "__main__":
    main()
