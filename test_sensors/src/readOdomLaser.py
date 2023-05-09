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
	ANGLE_VISION = 270

	FORBIDEN = 1
	DYNAMIC_WINDOW = 2
	FEASIBLE = 3
	ACTUAL = 4

	#Maximun Accelerations
	AMAXV = 0.7
	AMAXW = math.pi

	MAXDISTANCE = AMAXW/2

	SIZEROBOT = 0.15

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
		self.matrix = np.zeros((70,360))
		self.locRobot = np.array([-4.5, -6, 0])
		self.locObstacles = np.array([[-3.5, -6, 0]])
		self.sizeObstacles = np.array([[0.5, 0.5, 0.5]])

	
	def callback_teleop(self, msg: Twist):
		vel_stamped = TwistStamped()
		vel_stamped.header.stamp = rospy.Time.now()
		vel_stamped.twist = msg
		self.teleop_pub.publish(vel_stamped)

	def send_vel(self, v, w):
		self.vel_msg.linear.x = v
		self.vel_msg.angular.z = w																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																					
		self.vel_pub.publish(self.vel_msg)

	def hom(self,x: np.array):
		return np.array([
			[np.cos(x[2]), -np.sin(x[2]), x[0]],
			[np.sin(x[2]),  np.cos(x[2]), x[1]],
			[			0,			   0,    1]
		])

	def loc(self,mat: np.array):
		return np.array([mat[0][2], mat[1][2], math.atan2(mat[1][0], mat[0][0])])
 
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
	
	#Next Position of the robot in the world with the velocities v and w
	def nextPosition(self, v: float, w: float):
		self.lock.acquire()
		dt = self.timeCall
		self.lock.release()
		newX, newY, newTh = 0.0, 0.0, 0.0

		if abs(w) > 0.001:
			newX = self.locRobot[0] + (v / w) * (np.sin(self.locRobot[2] + w * dt) - np.sin(self.locRobot[2]))
			newY = self.locRobot[1] + (v / w) * (np.cos(self.locRobot[2] + w * dt) - np.cos(self.locRobot[2]))
			newTh = self.locRobot[2] + w * dt
		else:
			newX = self.locRobot[0] + dt * v * np.cos(self.locRobot[2])
			newY = self.locRobot[1] + dt * v * np.sin(self.locRobot[2])
			newTh = self.locRobot[2] + w * dt
		return np.array([newX, newY, newTh])
	
	def polar2Cartesian(self, distance: np.array):
		return np.array([distance[0] * np.cos(distance[1]), distance[0] * np.sin(distance[1]), distance[1]])
	
	# Minimun Distance from the robot to the most proximate obstacle
	def dist(self, v: float, w: float, polarObstacle: np.array):
		obstacle = self.polar2Cartesian(polarObstacle)

		self.lock.acquire()
		pos1 = self.locRobot
		self.lock.release()

		pos2 = self.nextPosition(v, w)


		#Positions of the robot and the obstacle in the world
		wTR2 = self.hom(pos2)
		wTR1 = self.hom(pos1)
		r1TO = self.hom(obstacle)

		#Position relative of the obstacle
		r2TO = np.linalg.inv(wTR2) @ wTR1 @ r1TO


		rXO = self.loc(r2TO)

		if rXO[1] == 0 and rXO[0] != 0:
			return rXO[0], 1
		elif rXO[0] == 0 and rXO[1] != 0:
			return rXO[1], 1
		else:
			radius = (rXO[0]**2 + rXO[1]**2)/ (2 * rXO[1])
			theta = math.atan2(2 * rXO[0] * rXO[1], rXO[0]**2 - rXO[1]**2)

			return radius * theta, 1 if radius > 0 else -1


		
	

	def velocities(self, vAct: float, wAct: float, vMin: float, vMax: float, wMin: float, wMax: float, scan_sub: np.array):
		vChange = 0.05
		wChange = 5 * math.pi / 180.0

		x = int(round((vMax - vMin) / vChange, 0)) + 1
		y = int(round((wMax - wMin) / wChange, 0)) + 1
		dWA = np.zeros([x,y])
		t1 = time.clock_gettime(time.CLOCK_REALTIME)
		i = 0
		j = 0

		# Dynamically admissible velocities
		limitVLow = vAct - self.AMAXV * self.timeCall
		limitVUp = vAct + self.AMAXV * self.timeCall
		limitWLow = wAct - self.AMAXW * self.timeCall
		limitWUp = wAct + self.AMAXW * self.timeCall

		obstacle = False

		for v in np.arange(vMax, vMin, -vChange):
			v = 0 if v > 0 and v < 0.01 else round(v, 2)
			for w in np.arange(wMin, wMax + wChange, wChange):
				w = 0 if abs(w) < 0.001 and abs(w) > 0 else round(w,3)
				# Actual Velocity
				if (v == vAct and w == wAct):
					dWA[i][j] = self.ACTUAL
				obstacle = False
				for d in scan_sub:
					# Maximum distance that limitW can be lower than w, the maximum distance thar limitV can be lower than v
					# is minor.
					if d[0] < self.MAXDISTANCE:
						obstacle = True
						distance, sign = TestNode.dist(self, v, w, d)
						distance = distance - self.SIZEROBOT

						if distance < 0:
							distance = 0.0

						# Admissible velocities
						# v <= (2 * distance(v, w) * vb ) ^ 1/2
						# w <= (2 * distance(v,w) * wb ) ^ 1/2

						limitV = math.sqrt(2.0 * distance * self.AMAXV)
						limitW = math.sqrt(2.0 * distance * self.AMAXW) * sign

						wAdmissible = True
						if limitW >= 0 and w >= 0:
							wAdmissible = w <= limitW
						elif limitW < 0 and w < 0:
							wAdmissible = w >= limitW


						# Feasible velocities
						if (v >= limitVLow and v <= limitVUp and w >= limitWLow and w <= limitWUp and v <= limitV and wAdmissible and dWA[i][j]!= self.ACTUAL and dWA[i][j] != self.FORBIDEN):
							dWA[i][j] = self.FEASIBLE
						# Dynamic Window
						elif (v >= limitVLow and v <= limitVUp and w >= limitWLow and w <= limitWUp and (dWA[i][j] == 0 or dWA[i][j] == self.DYNAMIC_WINDOW or dWA[i][j] == self.FORBIDEN)):
							dWA[i][j] = self.DYNAMIC_WINDOW
						# Forbidden Velocities
						elif ((v > limitV or not(wAdmissible)) and (dWA[i][j] != self.ACTUAL and dWA[i][j] != self.DYNAMIC_WINDOW)):
							#print("V: ", v, " limitV: ", limitV, " w: ", w, " limitW: ",limitW, " distance: ", distance )
							dWA[i][j] = self.FORBIDEN

				if not obstacle:
					distance = 30

					limitV = math.sqrt(2.0 * distance * self.AMAXV)
					limitW = math.sqrt(2.0 * distance * self.AMAXW)

					wAdmissible = True
					if limitW >= 0:
						wAdmissible = w <= limitW
					else:
						wAdmissible = w >= limitW

					# Feasible velocities
					if (v >= limitVLow and v <= limitVUp and w >= limitWLow and w <= limitWUp and v <= limitV and wAdmissible and dWA[i][j]!= self.ACTUAL and dWA[i][j] != self.FORBIDEN):
						dWA[i][j] = self.FEASIBLE
					# Dynamic Window
					elif (v >= limitVLow and v <= limitVUp and w >= limitWLow and w <= limitWUp and (dWA[i][j] == 0 or dWA[i][j] == self.DYNAMIC_WINDOW)):
						dWA[i][j] = self.DYNAMIC_WINDOW
					# Forbidden Velocities
					elif ((v > limitV or not(wAdmissible)) and (dWA[i][j] != self.ACTUAL and dWA[i][j] != self.DYNAMIC_WINDOW)):
						dWA[i][j] = self.FORBIDEN


				j += 1

			j = 0
			i += 1
		self.matrix = dWA
		t2 = time.clock_gettime(time.CLOCK_REALTIME) - t1
		#self.timeCall = t2
		# print(t2)
		return dWA
	
	def startPlot(self):
		self.p = Process(target=self.plotDWA(), args=())
		self.p.start()

	def plotDWA(self):
		fig, ax = plt.subplots()

		y = np.arange(0.8,-0.1, -0.1)
		y = np.round(y, 2)
		x = np.arange(-np.pi-(2*np.pi/7), np.pi+(2*np.pi/7), (2*np.pi/7))
		x = np.round(x, 3)

		while(True):
			ax.set_title("DWA")

			self.lock.acquire()
			ax.matshow(self.matrix)
			self.lock.release()

			ax.set_xticklabels(x)
			ax.set_yticklabels(y)

			fig.set_size_inches(10, 10, forward=True)
			fig.show()
			plt.pause(0.1)

	def searchVelocities(self, desired_v: float, desired_w: float, velocities):
		l = len(velocities)
		desired_w = 0 if desired_w == 0.0 else desired_w

		vx = desired_v
		wx = desired_w
		found, positive, negative, zero = False, False, False, False
		while(not found):
			try:
				ind = velocities.index([vx, wx])
				found = True
			except:
				if wx > -3.14 and not negative:
					wx -= round( 5 * math.pi /180.0, 3)
				elif wx == -3.14 and not positive:
					negative = True
					wx = desired_w + round(5 * math.pi/180.0, 3)
				elif wx < 3.14 and not positive:
					wx += round(5 * math.pi/180, 3)
				elif wx == 3.14 and not negative:
					positive = True
					wx = desired_w
				elif positive and negative and vx > 0:
					vx -= 0.05
					vx = round(vx, 2)
				elif positive and negative and vx == 0:
					zero = True
					vx = desired_v + 0.05
				elif positive and negative and zero:
					vx += 0.05


		return velocities[ind][0], velocities[ind][1]
		

	def callback(self, scan: LaserScan, odom: Odometry, desired_vel: TwistStamped):
		x = odom.pose.pose.position.x
		y = odom.pose.pose.position.y
		th = odom.pose.pose.orientation.z

		self.locRobot = np.array([x, y, th])
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

		distanceAngles = 2.5

		th = np.arange(-self.ANGLE_VISION/2, self.ANGLE_VISION/2 + distanceAngles, distanceAngles)
		 

		scan_sub = np.array([[np.amin(scan_ranges[(k-10):k]), th[int((k/10) - 1)]] for k in np.arange(10, len(scan_ranges) + 19, 10)])

		scan_sub = np.array([[s[0], np.deg2rad(s[1])] for s in scan_sub])

		# ¿Pongo las velocidades máximas o las deseadas?
		vMin = 0.0
		wMin = -math.pi
		vMax = 0.7
		wMax = math.pi


		# Vs = np.array([[0 for w in np.arange(wMin, wMax, math.pi/180.0)] for v in np.arange(vMin, vMax, 0.01)])
		

		arrV= TestNode.velocities(self, vAct, wAct, vMin, vMax, wMin, wMax, scan_sub)


		TestNode.searchVelocities(self, desired_v, desired_w, arrV)

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
