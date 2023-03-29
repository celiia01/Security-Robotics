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
from multiprocessing import Lock, Process


class TestNode:
	def __init__(self):
		self.lidar_sub = message_filters.Subscriber('base_scan', LaserScan)
		# figure = plt.figure(1)
		# ax = figure.add_subplot(111)
		# plt.show(block=False)
		# plt.pause(0.01)
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
		self.matrix = np.zeros((70,360))

	
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
		if (v > 0 and v < 0.001) or v == 0:
			distance = 30.0
			return distance, distance
		elif abs(w) > 0 and abs(w) < 0.0001 or w == 0:
			distance = np.amin(middle)
			# print("distance middle: ", np.amin(middle))
			return distance - v * dt, distance 
		elif w > 0.0001:
			distance = np.amin(middle) if np.amin(middle) < np.amin(bottom) else np.amin(bottom)
			#print("middle: ", np.amin(middle), ", bottom: ", np.amin(bottom))
		else:
			distance = np.amin(middle) if np.amin(middle) < np.amin(top) else np.amin(top)
			#print("middle: ", np.amin(middle), ", top: ", np.amin(top))

		return distance - (v/w * (w * dt)), distance
	
	# OPTIMIZAR BUCLE!!!!
	def velocities(self, middle: np.array, top: np.array, bottom: np.array, 
		vAct: float, wAct: float, vMin: float, vMax: float, wMin: float, wMax: float):
		aceleMaxV = 0.7
		aceleMaxW = math.pi
		vChange = 0.01
		wChange = math.pi /180.0

		Vr = []
		x = int((vMax - vMin) / vChange)
		y = int((wMax - wMin) / wChange)
		Vr1 = np.zeros([x,y])
		t1 = time.clock_gettime(time.CLOCK_REALTIME)
		i = 0
		j = 0
		limitVLow = vAct - aceleMaxV * self.timeCall
		limitVUp = vAct + aceleMaxV * self.timeCall
		limitWLow = wAct - aceleMaxW * self.timeCall
		limitWUp = wAct + aceleMaxW * self.timeCall


		for v in np.arange(vMax, vMin, -vChange):
			v = 0 if v > 0 and v < 0.01 else round(v, 2)
			for w in np.arange(wMin, wMax, wChange):
				
				#print("v: ", v, ", w: ", w)
				w = 0 if abs(w) < 0.001 and abs(w) > 0 else round(w,3)
				distance, distance0 = TestNode.dist(self, v, w, middle,top,bottom)
				distance -= 0.15
				distance = 0 if distance < 0 else distance
				# print("v: ", v, " w: ",w, " distance: ", distance)

				#print(acelBreakV)

				limitV = math.sqrt(2.0 * distance * aceleMaxV)
				limitW = math.sqrt(2.0 * distance * aceleMaxW)

				if (v > limitV or w > limitW):
					Vr1[i][j] = 1
				if (v >= limitVLow and v <= limitVUp and w >= limitWLow and w <= limitWUp):
					Vr1[i][j] = 2
				if (v >= limitVLow and v <= limitVUp and w >= limitWLow and w <= limitWUp and v <= limitV and w <= limitW):
					Vr.append([v,w])
					Vr1[i][j] = 3
				j += 1
			j = 0
			i += 1
			Vr1[0][1] = 1
			Vr1[0][2] = 2
			Vr1[0][3] = 3
		self.matrix = Vr1
		t2 = time.clock_gettime(time.CLOCK_REALTIME) - t1
		return Vr, Vr1
	
	def startPlot(self):
		self.p = Process(target=self.plotDWA(), args=())
		self.p.start()

	def plotDWA(self):
		fig, ax = plt.subplots()

		y = np.arange(0.8,0, -0.1)
		y = np.round(y, 2)
		x = np.arange(-np.pi-(2*np.pi/7), np.pi+(2*np.pi/7), (2*np.pi/7))
		x = np.round(x, 3)

		while(True):
			ax.cla()
			ax.set_title("DWA")

			self.lock.acquire()
			ax.matshow(self.matrix)
			self.lock.release()

			ax.set_xticklabels(x)
			ax.set_yticklabels(y)

			fig.show()
			plt.pause(0.1)

	def searchVelocities(self, desired_v: float, desired_w: float, velocities):

		pass
		

	def callback(self, scan: LaserScan, odom: Odometry, desired_vel: TwistStamped):
		x = odom.pose.pose.position.x
		y = odom.pose.pose.position.y
		# Desired Velocities
		desired_v = desired_vel.twist.linear.x																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																							
		desired_w = desired_vel.twist.angular.z
		# Actual Velocities (deseada)
		vAct = odom.twist.twist.linear.x if odom.twist.twist.linear.x > 0.0 or odom.twist.twist.linear.x < 0.0 else desired_v
		wAct = odom.twist.twist.angular.z if odom.twist.twist.angular.z > 0.0 or odom.twist.twist.angular.z < 0.0 else desired_w
		
		distance = 0.65
		scan_ranges = np.array(scan.ranges)
		middle = np.array(scan_ranges[478:600])
		top = np.array(scan_ranges[601:])
		bottom = np.array(scan_ranges[:477])

		# ¿Pongo las velocidades máximas o las deseadas?
		vMin = 0.0
		wMin = -math.pi
		vMax = 0.7
		wMax = math.pi


		# Vs = np.array([[0 for w in np.arange(wMin, wMax, math.pi/180.0)] for v in np.arange(vMin, vMax, 0.01)])
		

		_,Vr = TestNode.velocities(self, middle, top, bottom, vAct, wAct, vMin, vMax, wMin, wMax)

		self.matrix = Vr

		# v, w = TestNode.skipObstacle(self,scan_ranges, middle, top, bottom, desired_v, desired_w, distance)
		
		#print(middle)

		#print(len(scan.ranges), desired_v)																																																					
		
		v  = desired_v
		w = desired_w
		self.send_vel(v,w)
		#rospy.loginfo(rospy.get_caller_id() + "Min range %f", np.minimum(scan_ranges))

def main():
    rospy.init_node("test_node", anonymous=True)

    test_node = TestNode()

    test_node.startPlot()
    
    rospy.spin() 
    
	

if __name__ == "__main__":
    main()
