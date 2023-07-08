#!/usr/bin/python3.8

import rospy
import message_filters
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import time

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Twist
from multiprocessing import Lock, Process
from tf.transformations import euler_from_quaternion
from auxiliar import *



class TestNode:
	# Angle vision of the LaserScan
	ANGLE_VISION = 270

	# Types of velocities
	FREE = 0
	FORBIDEN = 1
	OUT = 2

	#Maximun Accelerations
	AMAXV = 0.7
	AMAXW = math.pi

	# Velocity constraints
	VMAX = 0.7
	VMIN = 0
	WMAX = math.pi
	WMIN = -math.pi

	# Maximum distance for possible collision
	MAXDISTANCE = AMAXW/2

	SIZEROBOT = 0.2

	# Value of the difference between the velocities chosen
	VCHANGE = 0.05
	WCHANGE = 15 * math.pi / 180.0

	# Pair of previous speeds
	VBEF = 0.0
	WBEF = 0.0

	# NUmber of values of the matrix
	Y = (WMAX - WMIN) / WCHANGE
	X = (VMAX - VMIN) / VCHANGE

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
		self.timeCall = 0.2
		#self.matrix = np.zeros((70,25))
		self.matrix = np.zeros((7,7))
		self.locRobot = np.array([-4.5, -6, 0])
		self.wake = False
		self.minV, self.maxV = 0.0, 0.7
		self.minW, self.maxW = -3.142, 3.142

	
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
	
	def velocityAdmissible(self, polar, positionAct):
		# TRABSFORM THE POLAR DSITANCES TO THE CARTESIANS ONE
		r1XO = self.polar2Cartesian(polar)

		theta = 1
		# CALCULO LAS DISTANCIAS A LOS DIFERENTES PUNTOS DEL OBSTACULO EN LA SIGUIENTE POSICIÓN DEL ROBOT
		if r1XO[0] == 0 and r1XO[1] == 0:
			radius = np.inf
			distance = 0
		#SI Y ES IGUAL A 0, NO  HAY ARCO 
		elif r1XO[1] == 0:
			radius = np.inf
			distance = r1XO[0] - self.SIZEROBOT

		#SI X ES IGUAL A 0
		elif r1XO[0] == 0:
			radius = r1XO[1]/2
			theta = math.atan2(0, r1XO[1]**2)
			distance = abs(radius * theta) -  self.SIZEROBOT
		else:
			#CALCULO EL RADIO Y LA THETA PARA ALCANZAR DICHA POSICIÓN
			radius = ((r1XO[0]) ** 2 + (r1XO[1])**2) / (2 * r1XO[1])
			theta = math.atan2((2 * r1XO[0] * r1XO[1]), (r1XO[0]**2 - r1XO[1]**2))
			distance = abs(radius * theta) - self.SIZEROBOT
		
		pairVelocities = []
		if distance < 0:
			for v in np.arange(self.VMAX, self.VMIN, -self.VCHANGE/2):
				if v != self.VMIN:
					pairVelocities.append([round(v,2), round(v/radius, 3)])

			return pairVelocities
		
		maximumV = math.sqrt(2.0 * distance * self.VMAX)
		w = maximumV/radius

		cociente = int(maximumV/self.VCHANGE)
		maxV = round(cociente * self.VCHANGE, 2)
		# print("maxV: ", maxV)
		
		if maxV <= self.VMAX:
			pairVelocities.append([maxV, w])
			pairVelocities.append([maxV, (maxV/(radius + self.SIZEROBOT))])
			pairVelocities.append([maxV, (maxV/(radius - self.SIZEROBOT))])

			for v in np.arange(self.VMAX, maxV, -self.VCHANGE/2):
				pairVelocities.append([round(v,2), round(v/radius,3)])
				pairVelocities.append([round(v,2), round(v/(radius + self.SIZEROBOT),3)])
				pairVelocities.append([round(v,2), round(v/(radius - self.SIZEROBOT),3)])

		
		return pairVelocities

	

	def restrictionDWA(self, v, w, admissible, limitVLow, limitVUp, limitWLow, limitWUp):
		'''
			Return the type of velocities for each one
		'''
		# Feasible velocities
		if (admissible and v >= limitVLow and v <= limitVUp and w >= limitWLow and w <= limitWUp ):
			return self.FREE
		# Dynamic Window
		elif (not admissible and v >= limitVLow and v <= limitVUp and w >= limitWLow and w <= limitWUp):
			return self.FORBIDEN
		# Forbidden Velocities
		elif not admissible:
			return self.FORBIDEN
		else:
			return self.FREE
	
	def velocities(self, vAct: float, wAct: float, scan_sub: np.array):
		# Dynamically admissible velocities
		limitVLow = round(vAct - self.AMAXV * self.timeCall, 2)
		limitVUp = round(vAct + self.AMAXV * self.timeCall, 2)
		limitWLow = round(wAct - self.AMAXW * self.timeCall, 3)
		limitWUp = round(wAct + self.AMAXW * self.timeCall, 3)


		minWCasilla = int((self.Y / 2) + round(limitWLow / self.WCHANGE))
		maxWCasilla = int((self.Y / 2) + round(limitWUp / self.WCHANGE))
		maxVCasilla = int((round(self.VMAX/self.VCHANGE)) - round(limitVLow / self.VCHANGE))
		minVCasilla = int((round(self.VMAX/self.VCHANGE)) - round(limitVUp / self.VCHANGE))

		x = int(round((maxVCasilla - minVCasilla), 0)) + 1
		y = int(round((maxWCasilla - minWCasilla), 0)) + 1
		dWA = np.zeros([x,y])

		i = 0
		if minVCasilla < 0:
			for h in range(minVCasilla,0):
				for j in range(0,y):
					dWA[i][j] = self.OUT
				
				i += 1

		i = x - 1
		if maxVCasilla > (round(self.VMAX/self.VCHANGE)):
			for h in range(int(round(self.VMAX/self.VCHANGE)), maxVCasilla):
				for j in range(0,y):
					dWA[i][j] = self.OUT
				i -= 1

		j = 0
		if minWCasilla < 0:
			for h in range(minWCasilla, 0):
				for i in range(0,x):
					dWA[i][j] = self.OUT
			j += 1

		j = y - 1
		if maxWCasilla > int(self.Y / 2):
			for h in range(int(self.Y / 2), maxWCasilla):
				for i in range(0, x):
					print(i,j)
					dWA[i][j] = self.OUT
			j -= 1
	
		self.lock.acquire()
		self.maxV, self.minV = self.VMAX - minVCasilla * self.VCHANGE, self.VMAX - maxVCasilla * self.VCHANGE
		self.minW, self.maxW = (minWCasilla -(self.Y / 2)) * self.WCHANGE, (maxWCasilla - (self.Y / 2)) * self.WCHANGE
		#POSITION ACTUAL
		pos1 = self.locRobot
		dt = self.timeCall
		self.lock.release()

		t1 = time.clock_gettime(time.CLOCK_REALTIME)

		for d in scan_sub:
			pairVelocities = self.velocityAdmissible(d, pos1)

			for velocity in pairVelocities:
				casillaW = (self.Y / 2) + round(velocity[1] / self.WCHANGE)
				casillaV = (round(self.VMAX/self.VCHANGE)) - round(velocity[0] / self.VCHANGE)
				if casillaV <= maxVCasilla and casillaV >= minVCasilla and casillaW <= maxWCasilla and casillaW >= minWCasilla:
					dWA[int(casillaV-minVCasilla)][int(casillaW-minWCasilla)] = self.FORBIDEN
				
		self.lock.acquire()
		self.matrix = dWA
		self.lock.release()
		t2 = time.clock_gettime(time.CLOCK_REALTIME) - t1
	
	
	def startPlot(self):
		self.p = Process(target=self.plotDWA(), args=())
		self.p.start()

	def plotDWA(self):
		fig, ax = plt.subplots()
		minV, maxV, minW, maxW = 0.0 ,0.0, 0.0, 0.0
		custom = colors.ListedColormap(["white", "red", "black"])
	

		while(True):
			t10 = time.clock_gettime(time.CLOCK_REALTIME)
			self.lock.acquire()
			minV, maxV = self.minV, self.maxV
			minW, maxW = self.minW, self.maxW
			self.lock.release()

			y = np.arange(maxV + self.VCHANGE, minV - self.VCHANGE, -self.VCHANGE)
			y = np.round(y, 2)
			x = np.arange(minW - self.WCHANGE, maxW  + self.WCHANGE, self.WCHANGE)
			x = np.round(x, 3)

			# self.lock.acquire()
			# print("								 mat[8][12]: ", self.matrix[8][12])
			# self.lock.release()
			ax.set_title("DWA")

			self.lock.acquire()
			ax.matshow(self.matrix, cmap=custom, vmin = self.FREE, vmax= self.OUT)
			#plt.heatmap(self.matrix, cmap=custom, vmin = self.FREE, vmax= self.ACTUAL)
			self.lock.release()
			# plt.
			ax.set_xticklabels(x)
			ax.set_yticklabels(y)

			#fig.set_size_inches(10, 10, forward=True)
			fig.show()
			plt.pause(0.001)

			self.lock.acquire()
			self.wake = True
			self.lock.release()
			#print("tD: ",time.clock_gettime(time.CLOCK_REALTIME) - t10)


	def searchVelocities(self, desired_v: float, desired_w: float, posibilities: np.array):
		'''
			Search the most proximate velocities to the desired velocities
		'''
		self.lock.acquire()
		dt = self.timeCall
		maxV = self.maxV
		minW = self.minW
		self.lock.release()

		# LINE AND COLUMN OF THE DESIRED VELOCITY IN THE SPACE OF SPEEDS
		line = int(np.round((self.VMAX - desired_v) / self.VCHANGE))
		colDec = ( desired_w + self.WMAX) / self.WCHANGE
		col = int(np.round(colDec))

		# LINE AND COLUMN OF THE MINIMUM VELOCITIES IN THE SPACE OF SPEEDS
		lineAct = int(np.round((self.VMAX - maxV) / self.VCHANGE))
		colDecAct = ( minW + self.WMAX) / self.WCHANGE
		colAct = int(np.round(colDecAct))

		limitVLow = round(self.VBEF - self.AMAXV * self.timeCall, 2)
		limitVUp = round(self.VBEF + self.AMAXV * self.timeCall, 2)
		limitWLow = round(self.WBEF - self.AMAXW * self.timeCall, 3)
		limitWUp = round(self.WBEF + self.AMAXW * self.timeCall, 3)

		windowSize = 0

		foundAllowed = False

		distancesToDesired = []
		positions = []

		
		# SEARCH A VELOCITY CLOSE TO THE DESIRED VELOCITY
		windowSize = 2

		vRet, wRet = 0.0, 0.0
			

		window = posibilities

		try:
			index = np.where(window==self.FREE)
			# CHANGE THE POSITIONS RELATIVE TO POSITIONS ABSOLUTES
			for i in range(len(index[0])):
				positionAbsoluteLine = index[0][i] + lineAct
				positionAbsoluteCol = index[1][i] + colAct
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

			if vRet > limitVUp:
				vRet = limitVUp
			elif vRet < limitVLow:
				vRet = limitVLow
				
			if wRet > limitWUp:
				wRet = limitWUp
			elif wRet < limitWLow:
				wRet = limitVLow

			if vRet == 0.0 and wRet == 0.0 and (desired_v != 0 or desired_w != 0):
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
		
		if not foundAllowed:
			print("FRENADA")
			vRet = round(self.VBEF - self.AMAXV * dt, 2)
			wRet = round(self.WBEF + self.AMAXW * dt, 3) if self.WBEF > 0 else round(self.WBEF - self.AMAXW * dt, 3)

			if wRet > math.pi:
				wRet = math.pi
			if wRet < -math.pi:
				wRet = -math.pi
		wRet = round(wRet, 3)
		vRet = 0 if vRet < 0 else vRet
		return vRet, wRet


		

	def callback(self, scan: LaserScan, odom: Odometry, desired_vel: TwistStamped):

		# print(np.arange(maxV, minV - self.VCHANGE, -self.VCHANGE))
		# print("						callback: ", scan.ranges[540])
		t5 = time.clock_gettime(time.CLOCK_REALTIME)
		x = odom.pose.pose.position.x
		y = odom.pose.pose.position.y
		ths = odom.pose.pose.orientation
		
		thsList = [ths.x, ths.y, ths.z, ths.w]
		(_, _, th) = euler_from_quaternion (thsList)

		self.locRobot = np.array([x, y, th])
		# Desired Velocities
		desired_v = desired_vel.twist.linear.x																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																							
		desired_w = desired_vel.twist.angular.z
		if desired_v != 0 or desired_w != 0:
		
			# Actual Velocities (deseada)
			# vAct = odom.twist.twist.linear.x if odom.twist.twist.linear.x > 0.0 or odom.twist.twist.linear.x < 0.0 else desired_v
			# wAct = odom.twist.twist.angular.z if odom.twist.twist.angular.z > 0.0 or odom.twist.twist.angular.z < 0.0 else desired_w
			
			#Wait until the another thread is wake
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

			distance = 1.4
			# Save the distances that the LaserScan catch
			scan_ranges = np.array(scan.ranges)
			# print(scan.ranges)
			# print("BEF: ", self.BEFORE)
			# print(scan.ranges == self.BEFORE)
			middle = np.array(scan_ranges[478:600])
			top = np.array(scan_ranges[601:])
			bottom = np.array(scan_ranges[:477])

			distanceAngles = 2.5


			th = np.arange(-self.ANGLE_VISION/2, self.ANGLE_VISION/2 + distanceAngles, distanceAngles)
			
			# Create a array with the minimum distance of the 10 positions and add the angle 
			scan_sub = np.array([[np.amin(scan_ranges[(k-10):k]), th[int((k/10) - 1)]] for k in np.arange(10, len(scan_ranges) + 19, 10)])
			
			t6 = time.clock_gettime(time.CLOCK_REALTIME)
			TestNode.velocities(self, self.VBEF, self.WBEF, scan_sub)

			self.lock.acquire()
			posibilities = self.matrix
			self.lock.release()

			v, w = TestNode.searchVelocities(self, desired_v, desired_w, posibilities)
			#print("t7: ", time.clock_gettime(time.CLOCK_REALTIME) - t6)
			self.VBEF = v
			self.WBEF = w
			print("selected: ", v, w)

			# v = desired_v
			# w = desired_w

			# v, w = TestNode.skipObstacle(self,scan_ranges, middle, top, bottom, desired_v, desired_w, distance)																																																				
			#print("t5: ", time.clock_gettime(time.CLOCK_REALTIME) - t5)
			self.BEFORE = scan.ranges
		else:
			v = desired_v
			w = desired_w
		# else:
		# v = self.VBEF
		# w = self.WBEF
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
