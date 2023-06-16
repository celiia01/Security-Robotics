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

	# Velocity constraints
	VMAX = 0.7
	VMIN = 0
	WMAX = math.pi
	WMIN = -math.pi

	MAXDISTANCE = AMAXW/2

	SIZEROBOT = 0.15

	VCHANGE = 0.05
	WCHANGE = 15 * math.pi / 180.0

	VBEF = 0.0
	WBEF = 0.0

	TRAYECTORIA_POSITIVA = False
	TRAYECTORIA_EMPEZADA = False

	BEFORE = []

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
		self.timeCall = 0.099
		self.matrix = np.zeros((70,25))
		self.locRobot = np.array([-4.5, -6, 0])
		self.wake = False

	
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
	
	# #Next Position of the robot in the world with the velocities v and w
	# def nextPosition(self, v: float, w: float, pos1):
	# 	self.lock.acquire()
	# 	dt = self.timeCall
	# 	self.lock.release()
	# 	newX, newY, newTh = 0.0, 0.0, 0.0

	# 	if abs(w) > 0.001:
	# 		newX = pos1[0] + (v / w) * (np.sin(pos1[2] + w * dt) - np.sin(pos1[2]))
	# 		newY = pos1[1] + (v / w) * (np.cos(pos1[2] + w * dt) - np.cos(pos1[2]))
	# 		newTh = pos1[2] + w * dt
	# 	else:
	# 		newX = pos1[0] + dt * v * np.cos(pos1[2])
	# 		newY = pos1[1] + dt * v * np.sin(pos1[2])
	# 		newTh = pos1[2] + w * dt
	# 	return np.array([newX, newY, newTh])

	def nextPosition(self, v: float, w: float, pos1):
		'''
			Return the new position of the robot depend of the first position
		'''
		self.lock.acquire()
		dt = self.timeCall
		self.lock.release()

		newX, newY = 0,0

		newTh = w * dt

		if abs(w) > 0.001:
			newX = (v/w) * np.sin(newTh)
			newY = (v/w) * (1 - np.cos(newTh))
		else:
			newX = v * dt * np.cos(newTh)
			newY = v * dt * np.sin(newTh)

		return np.array([newX, newY, newTh])


	
	def polar2Cartesian(self, distance: np.array):
		'''
			Return the Cartesian coordinates based on the polar coordinates

			x = r * cos(th)
			y = r * sin(th)
			th = th

		'''
		return np.array([distance[0] * np.cos(distance[1]), distance[0] * np.sin(distance[1]), distance[1]])
	
	
	# Minimun Distance from the robot to the most proximate obstacle
	def dist(self, v, w, polar):
		'''
			Return TRUE if the pair of velocities (v,w) are admissible for the obstacle, 
			else FALSE
		'''
		if len(polar) == 2 and polar[0][1] == np.deg2rad(-self.ANGLE_VISION / 2) and polar[1][1] == np.deg2rad(self.ANGLE_VISION / 2):
			print("Behind", v, w)
			return True
		#POSITION ACTUAL
		self.lock.acquire()
		pos1 = self.locRobot
		dt = self.timeCall
		self.lock.release()

		# CON V Y W SACO LA NUEVA POSICIÓN DONDE ESTARÁ EL ROBOT
		pos2 = self.nextPosition(v, w, pos1)
		r1TR2 = self.hom(pos2)

		
		obsDown = polar[0][1] < 0 and polar[len(polar) - 1][1] < 0
		obsUp = polar[0][1] > 0 and polar[len(polar) - 1][1] > 0
		obsInFront = polar[0][1] <= 0 and polar[len(polar) - 1][1] >= 0

		try:
			ind = np.where(polar[:][1] == 0)
			obsInFront = obsInFront and True
		except:
			obsInFront = False

		#PASO A COORDENADAS CARTESIANAS LAS COORDENADAS POLARES DE LOS OBSTACULOS
		cartesians = np.array([self.polar2Cartesian(d) for d in polar])
		rad1 = -np.inf
		rad2 =  np.inf

		allRad, allThetas = [], []
		allDistancesR2 = []


		r = v/w if w != 0 else np.inf

		for r1XO in cartesians:
			r1TO = self.hom(r1XO)
			# SACO LA POSICIÓN DEL OBSTACULO DEPENDIENDO DE LA NUEVA POSICIÓN DEL ROBOT
			r2TO = np.linalg.inv(r1TR2) @ r1TO
			r2XO = self.loc(r2TO)
			theta = 0
			# CALCULO EN LA POSICIÓN INICIAL QUE INTERVALOS DE VELOCIDADES PUEDEN SER TOMADAS
			if r1XO[0] == 0 and r1XO[1] == 0:
				radius = np.inf
				distance = 0
			#SI Y ES IGUAL A 0, NO  HAY ARCO 
			elif r1XO[1] == 0:
				radius = np.inf
				distance = r1XO[0]
			#SI X ES IGUAL A 0, NO HAY ARCO
			elif r1XO[0] == 0:
				radius = np.inf
				distance = r1XO[1]
			else:
				#CALCULO EL RADIO Y LA THETA PARA ALCANZAR DICHA POSICIÓN
				radius = ((r1XO[0]) ** 2 + (r1XO[1])**2) / (2 * r1XO[1])
				theta = math.atan2((2 * r1XO[0] * r1XO[1]), (r1XO[0]**2 - r1XO[1]**2))
				distance = abs(radius * theta)
			
					
			if obsInFront:
				if rad1 < radius:
					rad1 = radius
				if rad2 > radius:
					rad2 = radius
			elif obsUp:
				if distance < self.SIZEROBOT and w > 0:
					return False
				if rad1 < radius:
					rad1 = radius
				if rad2 > radius and radius >= 0:
					rad2 = radius
			elif obsDown:
				if distance < self.SIZEROBOT and w < 0:
					return False
				if rad1 < radius and radius <= 0:
					rad1 = radius
				if rad2 > radius:
					rad2 = radius

			allRad.append([radius])
			allThetas.append([theta])


			# CALCULO LAS DISTANCIAS A LOS DIFERENTES PUNTOS DEL OBSTACULO EN LA SIGUIENTE POSICIÓN DEL ROBOT
			if r2XO[0] == 0 and r2XO[1] == 0:
				radius = np.inf
				distance = 0
			#SI Y ES IGUAL A 0, NO  HAY ARCO 
			elif r2XO[1] == 0:
				radius = np.inf
				distance = r2XO[0] - self.SIZEROBOT

			#SI X ES IGUAL A 0, NO HAY ARCO
			elif r2XO[0] == 0:
				radius = np.inf
				distance = r2XO[1] - self.SIZEROBOT
			else:
				#CALCULO EL RADIO Y LA THETA PARA ALCANZAR DICHA POSICIÓN
				radius = ((r2XO[0]) ** 2 + (r2XO[1])**2) / (2 * r2XO[1])
				theta = math.atan2((2 * r2XO[0] * r2XO[1]), (r2XO[0]**2 - r2XO[1]**2))
				distance = abs(radius * theta) - self.SIZEROBOT
			
			if distance < 0 and ((obsInFront and ( (radius > 0 and w > 0) or (radius < 0 and w < 0) or w == 0))
									or (obsDown and w < 0) or (obsUp and w > 0)):
				return False
	
			allDistancesR2.append([distance])


		if len(allRad) > 0:
			arrayRad = np.asarray(allRad)
			arrayTh = np.asarray(allThetas)
			arrayDist = np.asarray(allDistancesR2)
			arrayDist = np.where(arrayDist >= 0, arrayDist, 100000)
			minorDistance = np.amin(arrayDist)

			#THE OBSTACLE IS IN FRONT THE ROBOT
			if obsInFront:
				if math.sqrt(2.0 * minorDistance * self.AMAXV) < v:
					return False

				newArrayr1 = np.where(arrayRad > 0, arrayRad, 100000)
				rad1 = np.amin(newArrayr1) - self.SIZEROBOT
				
				rad1Forbidden = (r >= rad1) if r > 0 else rad1 >= r

				newArrayr2 = np.where(arrayRad < 0, arrayRad, -100000)
				rad2 = np.amax(newArrayr2) + self.SIZEROBOT

				rad2Forbidden = (rad2 >= r) if r < 0 else rad2 <= r

				forbidden = rad1Forbidden and rad2Forbidden

				if forbidden and minorDistance != 100000:
					forbidden = ((minorDistance - self.AMAXV* dt) / 2) < v


			#THE OBSTACLE IS DOWN THE ROBOT
			elif obsDown:
				if w >0:
					return True
				if w == 0:
					arrayThetaPos = np.where(arrayTh >= 0, arrayTh, 100000)
					minorThetaPos = np.amin(arrayThetaPos)

					arrayThetaNeg = np.where(arrayTh <= 0, arrayTh, -100000)
					minorThetaNeg = np.amax(arrayThetaNeg)


					return minorThetaNeg < -self.WCHANGE and minorThetaPos > self.WCHANGE
		
				else:
					if math.sqrt(2.0 * minorDistance * self.AMAXV) < v:
						return False
					
					rad1 = (rad1 + self.SIZEROBOT) if (rad1 + self.SIZEROBOT) <= 0 else 0

					rad2 = (rad2 - self.SIZEROBOT) if rad2 < 0 else (rad2 + self.SIZEROBOT)
					
					rad1Forbidden = r <= rad1
					rad2Forbidden = r >= rad2


				forbidden = rad1Forbidden and rad2Forbidden

				if forbidden and minorDistance != 100000:
					forbidden = ((minorDistance - self.AMAXV* dt) / 2) < v
			#THE OBSTACLE IS UP THE ROBOT
			elif obsUp:
				if w < 0:
					return True
				if w == 0:
					arrayThetaPos = np.where(arrayTh >= 0, arrayTh, 100000)
					minorThetaPos = np.amin(arrayThetaPos)

					arrayThetaNeg = np.where(arrayTh <= 0, arrayTh, -100000)
					minorThetaNeg = np.amax(arrayThetaNeg)

					return minorThetaNeg < -self.WCHANGE and minorThetaPos > self.WCHANGE

				
				else:
					if math.sqrt(2.0 * minorDistance * self.AMAXV) < v:
						return False
					
					rad1 = (rad1 - self.SIZEROBOT) if (rad1 - self.SIZEROBOT) >= 0 else 0
				
					rad2 = (rad2 + self.SIZEROBOT)

					rad1Forbidden = r >= rad1
					rad2Forbidden = r <= rad2


				forbidden = rad1Forbidden and rad2Forbidden

				if forbidden and minorDistance != 100000:
					forbidden = ((minorDistance - self.AMAXV* dt) / 2) < v

			else:
				print("WARNING|||1, ", v, ", ", w)
				print("distances: ", polar)


		return not forbidden
		
			

	def restrictionDWA(self, v, w, admissible, limitVLow, limitVUp, limitWLow, limitWUp):
		'''
			Return the type of velocities for each one
		'''

		# Feasible velocities
		if (admissible and v >= limitVLow and v <= limitVUp and w >= limitWLow and w <= limitWUp ):
			return self.FEASIBLE
		# Dynamic Window
		elif (not admissible and v >= limitVLow and v <= limitVUp and w >= limitWLow and w <= limitWUp):
			return self.DYNAMIC_WINDOW
		# Forbidden Velocities
		elif not admissible:
			return self.FORBIDEN
		else:
			return 0
	

	def velocities(self, vAct: float, wAct: float, scan_sub: np.array):
		'''
		Calculate the DWA's matrix and return a matrix with 1's in the allowed pair of velocities and 0's in the forbidden ones
		'''
		self.VCHANGE = 0.05
		self.WCHANGE = 15 * math.pi / 180.0

		x = int(round((self.VMAX - self.VMIN) / self.VCHANGE, 0)) + 1
		y = int(round((self.WMAX - self.WMIN) / self.WCHANGE, 0)) + 1
		# print("x: ", x, ", y: ", y)
		dWA = np.zeros([x,y])
		posibilities = np.ones([x, y])
		t1 = time.clock_gettime(time.CLOCK_REALTIME)
		i = 0
		j = 0

		# Dynamically admissible velocities
		limitVLow = vAct - self.AMAXV * self.timeCall
		limitVUp = vAct + self.AMAXV * self.timeCall
		limitWLow = wAct - self.AMAXW * self.timeCall
		limitWUp = wAct + self.AMAXW * self.timeCall

		obstacle = False

		for v in np.arange(self.VMAX, self.VMIN -self.VCHANGE, -self.VCHANGE):
			v = 0 if v > 0 and v < 0.01 else round(v, 2)
			for w in np.arange(self.WMIN, self.WMAX + self.WCHANGE, self.WCHANGE):
				w = 0 if abs(w) < 0.001 and abs(w) > 0 else round(w,3)
				obstacle = False
				if v != 0 and len(scan_sub) != 0:
					t3 = time.clock_gettime(time.CLOCK_REALTIME)
					obstacle = True

					t3 = time.clock_gettime(time.CLOCK_REALTIME)
					admissible = True
					for obs in scan_sub:
						admissible = admissible and self.dist(v,w,obs)
	
					t4 = time.clock_gettime(time.CLOCK_REALTIME) - t3
					num = self.restrictionDWA(v, w, admissible, limitVLow, limitVUp, limitWLow, limitWUp)
					

					if num == self.FEASIBLE and dWA[i][j] != self.FORBIDEN and dWA[i][j] != self.DYNAMIC_WINDOW:
						dWA[i][j] = num
					elif num == self.DYNAMIC_WINDOW:
						dWA[i][j] = num
						posibilities[i][j] = 0
					elif num == self.FORBIDEN and dWA[i][j] != self.DYNAMIC_WINDOW:
						dWA[i][j] = num
						
						posibilities[i][j] = 0

					t4 = time.clock_gettime(time.CLOCK_REALTIME) - t3

				if not obstacle:

					num = self.restrictionDWA(v, w, True, limitVLow, limitVUp, limitWLow, limitWUp)
					

					if num == self.FEASIBLE and dWA[i][j] != self.FORBIDEN and dWA[i][j] != self.DYNAMIC_WINDOW:
						dWA[i][j] = num
						
					elif num == self.DYNAMIC_WINDOW:
						dWA[i][j] = num
						
						posibilities[i][j] = 0
					elif num == self.FORBIDEN and dWA[i][j] != self.DYNAMIC_WINDOW:
						dWA[i][j] = num
						posibilities[i][j] = 0
						

				# Actual Velocity
				if (v == vAct and w == wAct):
					dWA[i][j] = self.ACTUAL
				j += 1
					

			j = 0
			i += 1
		self.lock.acquire()
		self.matrix = dWA
		self.lock.release()
		t2 = time.clock_gettime(time.CLOCK_REALTIME) - t1
		return posibilities
	
	def startPlot(self):
		self.p = Process(target=self.plotDWA(), args=())
		self.p.start()

	def plotDWA(self):
		fig, ax = plt.subplots()
		self.WCHANGE = (2 * math.pi) / 5

		y = np.arange(0.8,-0.1, -0.1)
		y = np.round(y, 2)
		x = np.arange(-np.pi-self.WCHANGE, np.pi+self.WCHANGE, self.WCHANGE)
		x = np.round(x, 3)

		while(True):
			# self.lock.acquire()
			# print("								 mat[8][12]: ", self.matrix[8][12])
			# self.lock.release()
			ax.set_title("DWA")

			self.lock.acquire()
			ax.matshow(self.matrix)
			self.lock.release()

			ax.set_xticklabels(x)
			ax.set_yticklabels(y)

			#fig.set_size_inches(10, 10, forward=True)
			fig.show()
			plt.pause(0.1)
			self.lock.acquire()
			self.wake = True
			self.lock.release()
			time.sleep(0.001)

	# def searchVelocities(self, desired_v: float, desired_w: float, posibilities: np.array):
	# 	'''
	# 		Search the most proximate velocities to the desired velocities
	# 	'''
	# 	line = int(np.round((self.VMAX - desired_v) / self.VCHANGE))
	# 	lineBef = int(np.round((self.VMAX - self.VBEF) / self.VCHANGE))
	# 	colDec = ( desired_w + self.WMAX) / self.WCHANGE
	# 	col = int(np.round(colDec))
	# 	colDecBef = ( self.WBEF + self.WMAX) / self.WCHANGE
	# 	colBef = int(np.round(colDecBef))
	# 	h,w = posibilities.shape
	# 	ind = np.array([0,0])
	# 	vRet, wRet, indv, indw = 0, 0, 0, 0
	# 	minLine, minCol, maxCol, maxLine = 0, 0, 0, 0 
	# 	found = False
	# 	wind = 0
	
	# 	#print("line: ", line, " lineBEf: ", lineBef, ", col: ", col, " colBef: ", colBef)
	# 	if posibilities[line][col] == 0:

	# 		# if posibilities[lineBef][colBef] == 1:
	# 		# 	found = True

	# 		# line = lineBef
	# 		# colDec = colDecBef
	# 		# col = colBef
			
	# 		wind = 1

	# 		while not found and (minLine != 0 or maxLine != h + 1 or minCol != 0 or maxCol != w + 1):
	# 			#print("entro")
	# 			#Window with 2 * wind + 1 x 2 * wind + 1
	# 			minLine = line - wind if line - wind >= 0 else 0
	# 			maxLine = line + wind + 1 if line + wind <= h else h + 1
	# 			minCol = col - wind if col - wind >= 0 else 0
	# 			maxCol = col + wind + 1 if col + wind <= w else w + 1


	# 			newPos = posibilities[minLine : maxLine, minCol: maxCol]

	# 			try:
	# 				ind = np.where(newPos==1)
	# 				#print(ind)
	# 				indv = ind[0][0] - ( wind if wind < line else 0)
	# 				indw = ind[1][0] - ( wind if wind < col else 0)
	# 				found = True
	# 			except:
	# 				wind += 1
	# 	else:
	# 		#print("DESIRED TRUE")
	# 		found = True

	# 	if found:
	# 		lineRet = (line + indv) if wind < line  else indv
	# 		colRet = (colDec + indw) if wind < col else indw
	# 		vRet = np.round((self.VMAX - lineRet * self.VCHANGE), 2)
	# 		wRet = np.round(((colRet * self.WCHANGE) - self.WMAX), 3)

	# 		#print("line: ", lineRet , ", col: ", colRet)

	# 	return vRet, wRet

	def searchVelocities(self, desired_v: float, desired_w: float, posibilities: np.array):
		'''
			Search the most proximate velocities to the desired velocities
		'''
		self.lock.acquire()
		dt = self.timeCall
		self.lock.release()
		# LINE AND COLUMN OF THE DESIRED VELOCITY
		line = int(np.round((self.VMAX - desired_v) / self.VCHANGE))
		colDec = ( desired_w + self.WMAX) / self.WCHANGE
		col = int(np.round(colDec))

		# LINE AND COLUMN OF THE ACTUAL VELOCITY
		lineAct = int(np.round((self.VMAX - self.VBEF) / self.VCHANGE))
		colDecAct = ( self.WBEF + self.WMAX) / self.WCHANGE
		colAct = int(np.round(colDecAct))

		#SHAPE OF THE MATRIX
		h,w = posibilities.shape

		windowSize = 0

		foundAllowed = False

		minLine, minCol, maxCol, maxLine = 0, 0, 0, 0 

		distancesToDesired = []
		positions = []

		positionsAchievables =[ [lineAct - (1 if lineAct - 1 >= 0 else 0), colAct - (1 if colAct - 1 >= 0 else 0)],
		       				[lineAct - (1 if lineAct - 1 >= 0 else 0), colAct], 
							[lineAct - (1 if lineAct - 1 >= 0 else 0), colAct + (1 if colAct + 1 >= 0 else 0)],
							[lineAct, colAct - (1 if colAct - 1 >= 0 else 0)], [lineAct, colAct],
							[lineAct, colAct + (1 if colAct + 1 >= 0 else 0)],
							[lineAct + (1 if lineAct + 1 >= 0 else 0), colAct - (1 if colAct - 1 >= 0 else 0)],
							[lineAct + (1 if lineAct + 1 >= 0 else 0), colAct], 
							[lineAct + (1 if lineAct + 1 >= 0 else 0), colAct + (1 if colAct + 1 >= 0 else 0)]]

		
		# SEARCH A VELOCITY CLOSE TO THE DESIRED VELOCITY
		windowSize = 1

		vRet, wRet = 0.0, 0.0

		while not foundAllowed and (minLine != 0 or maxLine != h + 1 or minCol != 0 or maxCol != w + 1):
			
			# WINDOW WITH 2 * WINDOWSIZE + 1 X 2 * WINDOWSIZE + 1
			minLine = lineAct - windowSize if lineAct - windowSize >= 0 else 0
			maxLine = lineAct + windowSize + 1 if lineAct + windowSize <= h else h + 1
			minCol = colAct - windowSize if colAct - windowSize >= 0 else 0
			maxCol = colAct + windowSize + 1 if colAct + windowSize <= w else w + 1


			window = posibilities[minLine : maxLine, minCol: maxCol]

			try:
				index = np.where(window==1)
				# CHANGE THE POSITIONS RELATIVE TO POSITIONS ABSOLUTES
				for i in range(len(index[0])):
					positionAbsoluteLine = index[0][i] - windowSize + lineAct
					positionAbsoluteCol = index[1][i] - windowSize + colAct
					positions += [[positionAbsoluteLine, positionAbsoluteCol]]

					#CALCULATE THE DISTANCES BETWEEN THIS POINT AND THE POINT OF THE DESIRED VELOCITY
					distancesToDesired += [[math.sqrt((line - positionAbsoluteLine) **2+ (col - positionAbsoluteCol) ** 2)]]
				# SELECT THE POINT WITH THE MINIMUM DISTANCE TO THE SELECTED
				minimunDistance = np.amin(distancesToDesired)
				searchingIndex = np.where(distancesToDesired == minimunDistance)[0][0]
				selected = positions[searchingIndex]

				# VELOCITIES RELATED TO THE POSITION SELECTED
				vRet = np.round((self.VMAX - selected[0] * self.VCHANGE), 2)
				wRet = np.round(((selected[1] * self.WCHANGE) - self.WMAX), 3)
				if vRet == 0.0 and wRet == 0.0:
					distancesToDesired[searchingIndex] = [np.inf]
					# SELECT THE POINT WITH THE SECOND MINIMUM DISTANCE
					minimunDistance = np.min(distancesToDesired)
					searchingIndex = np.where(distancesToDesired == minimunDistance)[0][0]
					selected = positions[searchingIndex]

					# NEW VELOCITIES RELATED TO THE NEW POSITION SELECTED
					vRet = np.round((self.VMAX - selected[0] * self.VCHANGE), 2)
					wRet = np.round(((selected[1] * self.WCHANGE) - self.WMAX), 3)

			
				foundAllowed = True
			except:
				windowSize += 1
		
		if foundAllowed:

			# CHECK IF THE VELOCITY IS ACHIEVABLE
			linearAchievable = self.VBEF + self.AMAXV * dt >= vRet and self.VBEF - self.AMAXV * dt <= vRet
			angularAchievable = self.WBEF + self.AMAXW * dt >= wRet and self.WBEF - self.AMAXW * dt <= wRet
				
			if not linearAchievable and not angularAchievable:
				distanceToSelected = []
				# THE SELECTED VELOCITY IS NOT REACHABLE
				for dinamic in positionsAchievables:
					distanceToSelected += [[math.sqrt((selected[0] - dinamic[0])** 2 + (selected[1] - dinamic[1])** 2)]]

				# SELECT THE POINT WITH THE MINIMUM DISTANCE TO THE SELECTED
				minimunDistance = np.amin(distanceToSelected)
				searchingIndex = np.where(distanceToSelected == minimunDistance)[0][0]
				selected = positionsAchievables[searchingIndex]

				vRet = np.round((self.VMAX - selected[0] * self.VCHANGE), 2)
				wRet = np.round(((selected[1] * self.WCHANGE) - self.WMAX), 3)
				if vRet == 0.0 and wRet == 0.0:
					distanceToSelected[searchingIndex] = [np.inf]
					# SELECT THE POINT WITH THE SECOND MINIMUM DISTANCE
					minimunDistance = np.amin(distanceToSelected)
					searchingIndex = np.where(distanceToSelected == minimunDistance)[0][0]
					selected = positionsAchievables[searchingIndex]

					# NEW VELOCITIES RELATED TO THE NEW POSITION SELECTED
					vRet = np.round((self.VMAX - selected[0] * self.VCHANGE), 2)
					wRet = np.round(((selected[1] * self.WCHANGE) - self.WMAX), 3)


			return vRet, wRet


		

	def callback(self, scan: LaserScan, odom: Odometry, desired_vel: TwistStamped):
		# print("						callback: ", scan.ranges[540])
		t5 = time.clock_gettime(time.CLOCK_REALTIME)
		# if scan.ranges != self.BEFORE:
		
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
		
		self.lock.acquire()
		wake = self.wake
		self.lock.release()

		while not wake:
			self.VBEF = 0
			self.WBEF = 0
			self.lock.acquire()
			wake = self.wake
			self.lock.release()
			time.sleep(0.1)

		distance = 0.65
		scan_ranges = np.array(scan.ranges)
		# print(scan.ranges)
		# print("BEF: ", self.BEFORE)
		# print(scan.ranges == self.BEFORE)
		middle = np.array(scan_ranges[478:600])
		top = np.array(scan_ranges[601:])
		bottom = np.array(scan_ranges[:477])

		distanceAngles = 2.5

		# print("										>",scan_ranges[540])

		th = np.arange(-self.ANGLE_VISION/2, self.ANGLE_VISION/2 + distanceAngles, distanceAngles)
		

		scan_sub = np.array([[np.amin(scan_ranges[(k-10):k]), th[int((k/10) - 1)]] for k in np.arange(10, len(scan_ranges) + 19, 10)])

		scan_deletes, newObs = [], []
		last_deg = -self.ANGLE_VISION/2
		begin = False
		# for s in scan_sub:
		# 	if s[0] - self.SIZEROBOT<= self.MAXDISTANCE:
		# 		scan_deletes += [[s[0] - self.SIZEROBOT, np.deg2rad(s[1])]]

		for s in scan_sub:
			if s[0] - self.SIZEROBOT<= self.MAXDISTANCE:
				if not begin:
					newObs = []
					begin = True
				newObs += [[s[0] - self.SIZEROBOT, np.deg2rad(s[1])]]
			else:
				if begin:
					scan_deletes += [newObs]
					begin = False

		if len(newObs) > 0:
			scan_deletes += [newObs]
		


		posibilities = TestNode.velocities(self, self.VBEF, self.WBEF, scan_deletes)


		v, w = TestNode.searchVelocities(self, desired_v, desired_w, posibilities)
		print("v: ", v, ", w: ",w )
		self.VBEF = v
		self.WBEF = w

		# v = desired_v
		# w = desired_w

		# v, w = TestNode.skipObstacle(self,scan_ranges, middle, top, bottom, desired_v, desired_w, distance)																																																				
		# print("t5: ", time.clock_gettime(time.CLOCK_REALTIME) - t5)
		self.BEFORE = scan.ranges
		# else:
		# 	v = self.VBEF
		# 	w = self.WBEF
			# print("t5: ", time.clock_gettime(time.CLOCK_REALTIME) - t5)
		#input("continue")
		self.send_vel(v,w)
		#rospy.loginfo(rospy.get_caller_id() + "Min range %f", np.minimum(scan_ranges))

def main():
    rospy.init_node("test_node", anonymous=True)

    test_node = TestNode()

    test_node.startPlot()
    
    rospy.spin() 
    
	

if __name__ == "__main__":
    main()
