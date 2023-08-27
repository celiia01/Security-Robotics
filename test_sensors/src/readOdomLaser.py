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
from matplotlib.patches import Rectangle



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

	TIMECALL = 0.2

	# Maximum distance for possible collision
	MAXDISTANCE = AMAXW/2

	SIZEROBOT = 0.18

	# Value of the difference between the velocities chosen
	VCHANGE = (AMAXV * TIMECALL) / 3
	WCHANGE = (AMAXW * TIMECALL) / 3

	# Pair of previous speeds
	VBEF = 0.0
	WBEF = 0.0

	# NUmber of values of the matrix
	X = (WMAX - WMIN) / WCHANGE
	Y = (VMAX - VMIN) / VCHANGE

	# Distance of the previous point
	DISTANCEBEFORE = np.inf
	# Location of the previous obstacle
	LOCOBSBEFORE = []
	# Variable if it was not found a border
	ESQUINA = True
	# Distance between the angles
	DISTANCEANGLE = 0
	# Variable if it was not found the inferior border
	INFERIOR = True 
	# Angle result of the atan2
	THETATAN = 0
	# Radius of the previous trayectory
	LASTRADIUS = 0
	# Variable that indicate if the point is the last
	LASTDISTANCE = False


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
		self.matrix = np.zeros((7,7))
		self.locRobot = np.array([-4.5, -6, 0])
		self.wake = False
		self.minV, self.maxV = 0.0, 0.7
		self.minW, self.maxW = -3.142, 3.142
		self.casillaVMax, self.casillaVMin = 10, 0
		self.casillaWMax, self.casillaWMin = 10, 0

	
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
		'''
			First implementation
		'''
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

	def nextPosition(self, v: float, w: float):
		'''
			Return the new position of the robot depend of the first position
		'''

		newX, newY = 0,0

		newTh = w * self.TIMECALL

		if abs(w) > 0.001:
			newX = (v/w) * np.sin(newTh)
			newY = (v/w) * (1 - np.cos(newTh))
		else:
			newX = v * self.TIMECALL * np.cos(newTh)
			newY = v * self.TIMECALL * np.sin(newTh)

		return np.array([newX, newY, newTh])


	
	def polar2Cartesian(self, distance: np.array, r1XWAngle):
		'''
			Return the Cartesian coordinates based on the polar coordinates

			x = r * cos(th)
			y = r * sin(th)

		'''
		x = distance[0] * np.cos(distance[1])
		y = distance[0] * np.sin(distance[1])
		return np.array([x, y, r1XWAngle])
	
	def calculateDistance(self, r1XO):
		'''
			Calculate the distance in the arc, the radius of the trayectorie and the angle of the point r1XO

		'''
		# print("rXO INI DISTANCE: ", r1XO)
		theta = 0
		# CALCULO LAS DISTANCIAS A LOS DIFERENTES PUNTOS DEL OBSTACULO EN LA SIGUIENTE POSICIÓN DEL ROBOT
		if r1XO[0] == 0 and r1XO[1] == 0:
			radius = np.inf
			distance = 0
		#SI Y ES IGUAL A 0, NO  HAY ARCO 
		elif r1XO[1] == 0:
			radius = np.inf
			distance = abs(r1XO[0]) - self.SIZEROBOT

		#SI X ES IGUAL A 0
		elif r1XO[0] == 0:
			radius = r1XO[1]/2
			theta = math.atan2(0, r1XO[1]**2)
			distance = abs(radius * theta) - self.SIZEROBOT

		else:
			#CALCULO EL RADIO Y LA THETA PARA ALCANZAR DICHA POSICIÓN
			radius = ((r1XO[0]) ** 2 + (r1XO[1])**2) / (2 * r1XO[1])
			if abs(radius) < 0.01:
				radius = np.inf
				distance = abs(r1XO[0]) - self.SIZEROBOT
			else:
				theta = math.atan2((2 * r1XO[0] * r1XO[1]), ((r1XO[0])**2 - (r1XO[1])**2)) 
				distance = abs(radius * theta) - self.SIZEROBOT

		return radius, theta, distance
	
	
	def discrete(self, distance: float, rad: float):
		'''
			Return the maximum lineal velocity that the robot can stop safetly
		'''
		distTotal = 0
		for v in np.arange(self.VMIN, self.VMAX, self.VCHANGE):
			v = round(v,2)
			w = v/rad
			thetaRec = w * self.TIMECALL

			# Calculo la distancia recorrida en el arco
			if rad == np.inf:
				# No hay arcio
				distTotal += (v * self.TIMECALL)
			else:
				# Hay arco
				distTotal += abs(rad * thetaRec)
			# Si la distancia Total más el tamaño del robot es mayor o igual a la distancia del robot al punto
			if (distTotal + self.SIZEROBOT) >= distance:
				return v
			
		return 10
	
	def calculatePairVelocity(self, distance, radius, polar):
		'''
			Return a array with all pair of Velocities forbidden 
		'''
		pairVelocities = []
		angleInFront = 42.5

		# Si la distancia es menor que 0, hay peligro de colisión
		if distance < 0:
			for v in np.arange(self.VMIN + self.VCHANGE, self.VMAX, self.VCHANGE):
				w = v/radius

				#Si el signo de la velocidad angular y el radio son el mismo
				if (w <= 0 and radius <= 0) or (radius > 0 and w >= 0 ):
					negative = w < 0 
					# Dependiendo del signo se escogen un máximo y un mínimo
					if negative:
						min = self.WMIN
						max = -self.WCHANGE if polar[1] < np.deg2rad(-angleInFront) else 0
					else:
						min = self.WCHANGE if polar[1] > np.deg2rad(angleInFront) else 0
						max = self.WMAX
					
					for w in np.arange(min, max, self.WCHANGE):
						pairVelocities.append([v, round(w,3)])
						

		elif distance < 10:
			# Discrete
			vMax = self.discrete(distance, radius)

			w = vMax / radius

			# Si la velocidad angular está próxima al cero hay que comprobar
			# si el obstaculo representa un peligro
			if w < self.WCHANGE and w > -self.WCHANGE:
				if polar[1] < np.deg2rad(angleInFront) and polar[1] > np.deg2rad(-angleInFront):
					pairVelocities.append([vMax, round(w,3)])
			else:
				pairVelocities.append([vMax, round(w,3)])

			for v in np.arange(self.VMAX, vMax, -self.VCHANGE/2):
				w = v/radius

				#Velocidad angular con el anterior radio
				w2 = v/self.LASTRADIUS if self.LASTRADIUS != 0 else 0

				#Si el signo de la velocidad angular y el radio son el mismo
				if ((radius < 0 and w <= 0) or (radius > 0 and w >= 0)):
					
					# Si la velocidad angular está próxima al cero hay que comprobar
					# si el obstaculo representa un peligro
					if w < self.WCHANGE and w > -self.WCHANGE: 
						if polar[1] < np.deg2rad(angleInFront) and polar[1] > np.deg2rad(-angleInFront):
							pairVelocities.append([v, round(w,3)])
					else:
						pairVelocities.append([v, round(w,3)])
					wchange = round(self.WCHANGE/2 if w < w2 else -self.WCHANGE/2,3)
					
					#Se añaden todas las velocidades angulares comprendidas entre w y w2
					for w1 in np.arange(w, w2, wchange):
						
						if w1 < self.WCHANGE and w1 > -self.WCHANGE:
							if polar[1] < np.deg2rad(angleInFront) and polar[1] > np.deg2rad(-angleInFront):
								pairVelocities.append([v, round(w1,3)])
						else:
							pairVelocities.append([v, round(w1,3)])
			
		return pairVelocities
			
	
	def velocityAdmissible(self, polar):

		wXR1 = self.locRobot

		r1XW = loc(np.linalg.inv(hom(wXR1)))
		
		# TRABSFORM THE POLAR DSITANCES TO THE CARTESIANS  ONE
		r1XO = self.polar2Cartesian(polar, r1XW[2])

		r1TO = hom(r1XO)


		#Se calcula la distancia sin engordar el obstaculo
		radius, theta, distance = self.calculateDistance(r1XO)

		pairVelocities = []
		interR1XO2 = []

		#Si la distancia del punto y del anterior punto son menores que 30
		if round(distance) < 30 and round(self.DISTANCEBEFORE) < 30:
			
			# Se calcula la dirección que sigue el contorno del obstaculo y se suma pi/2 para sacar la perpendicular
			wXO = loc(hom(self.locRobot)@hom(r1XO))
			wXOb = loc(hom(self.locRobot)@hom(self.LOCOBSBEFORE))
			thetaAtan = norm(math.atan2(wXO[1] - wXOb[1], wXO[0] - wXOb[0]) + math.pi/2)
	
			#Con la dirección y el tamaño que queremos que se engorde el obstaculo se calcula un nuevo punto
			o1XO2 = self.polar2Cartesian(np.array([self.SIZEROBOT, thetaAtan]), 0)
			
			#Se calcula la locaclización del punto con respecto al robot
			r1XO2 = loc(r1TO @ hom(o1XO2))

			#Se consigue el radio, el ángulo y la distancia de la trayectoria hacia el nuevo punto
			radius, theta, distance = self.calculateDistance(r1XO2)

			# Si se ha localizado una esquina anteriormente pero la variable INFERIOR está a TRUE
			# Es la esquina inferior y ya se puede conseguir los datos necesarios para tratarla
			if not self.ESQUINA and self.INFERIOR:
				# Se consigue la dirección hacia donde se tiene que aumentar el tamaño
				thetaAtanE = thetaAtan + math.pi/4 
				#Con la dirección y el tamaño que queremos que se engorde el obstaculo se calcula un nuevo punto
				o1XO2 = self.polar2Cartesian(np.array([self.SIZEROBOT, thetaAtanE]), 0)
				# Se guarda la localización para posteriormente tratarla
				interR1XO2 = loc(hom(self.LOCOBSBEFORE) @ hom(o1XO2))
				self.INFERIOR = False

			#Esquina encontrada entre medio de un grupo de puntos con distancias menores a la máxima
			if not self.ESQUINA and abs(self.THETATAN - thetaAtan) > np.deg2rad(45):
				# Se consigue la dirección hacia donde se tiene que aumentar el tamaño
				thetaAtanE = thetaAtan + math.pi/4 
				#Con la dirección y el tamaño que queremos que se engorde el obstaculo se calcula un nuevo punto
				o1XO2 = self.polar2Cartesian(np.array([self.SIZEROBOT, thetaAtanE]), 0)
				# Se guarda la localización para posteriormente tratarla
				interR1XO2 = loc(hom(self.LOCOBSBEFORE) @ hom(o1XO2))
			self.THETATAN = thetaAtan
		# Se resetean variables ya que hay no hay más obstaculos en dicho espacio
		else:
			self.INFERIOR = True


		pairVelocities = self.calculatePairVelocity(distance, radius, polar)

		
		# #INFERIOR CORNER
		if self.ESQUINA and distance < 30 and self.DISTANCEBEFORE > 30:
			self.ESQUINA = False


		#SUPERIOR BORDER 
		elif not self.ESQUINA and ((distance > 30 and self.DISTANCEBEFORE < 30) or self.LASTDISTANCE):

			# Se consigue la dirección hacia donde se tiene que aumentar el tamaño
			thetaAtan = self.THETATAN - math.pi/4

			#Con la dirección y el tamaño que queremos que se engorde el obstaculo se calcula un nuevo punto
			o1XO2 = self.polar2Cartesian(np.array([self.SIZEROBOT, thetaAtan]), 0)
			# Se guarda la localización para posteriormente tratarla
			interR1XO2 = loc(hom(self.LOCOBSBEFORE) @ hom(o1XO2))
			# Como se ha encontrado un punto mayor que la distancia mínina se resetea la variable
			self.ESQUINA = True


		
		self.DISTANCEBEFORE = distance
			
		if len(interR1XO2) != 0:
			# Se calcula el radio, el angulo y la distancia de la esquina
			rad, theta, distance = self.calculateDistance(interR1XO2)

			# Se añade las velocidades no admisibles de la esquina a las del punto tratado
			pairVelocities = pairVelocities + self.calculatePairVelocity(distance, rad, polar)


		self.LOCOBSBEFORE = r1XO
		self.LASTRADIUS = radius

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
	
	def spaceVelocities(self, vAct: float, wAct: float, scan_sub: np.array):
		'''
			Complete the matrix that represent the space of Velocities
		'''
		# Dynamically admissible velocities
		limitVLow = round(vAct - self.AMAXV * self.TIMECALL, 2)
		limitVUp = round(vAct + self.AMAXV * self.TIMECALL, 2)
		limitWLow = round(wAct - self.AMAXW * self.TIMECALL, 3)
		limitWUp = round(wAct + self.AMAXW * self.TIMECALL, 3)

		# Casillas que contienen las velocidades dinámicas calculadas anteriormente
		minWCasilla = int((self.X / 2) + round(limitWLow / self.WCHANGE))
		maxWCasilla = int((self.X / 2) + round(limitWUp / self.WCHANGE))
		maxVCasilla = int((round(self.VMAX/self.VCHANGE)) - round(limitVLow / self.VCHANGE))
		minVCasilla = int((round(self.VMAX/self.VCHANGE)) - round(limitVUp / self.VCHANGE))

		# Tamaño de la matriz
		# Ventana
		x = int(round((maxVCasilla - minVCasilla), 0)) + 1
		y = int(round((maxWCasilla - minWCasilla), 0)) + 1
		# Todo el rango de velocidades
		# x = int(round((self.VMAX)/ self.VCHANGE, 0)) + 1
		# y = int(round((self.WMAX * 2)/self.WCHANGE, 0)) + 1
		dWA = np.zeros([x,y])

		# Bucles para comprobar si hay alguna velocidad fuera del rango de velocidades permitidas
		# Si se quiere mostrar todo el rango de velocidades comentarlo
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

		if maxWCasilla > self.X:
			for h in range(int(self.X), maxWCasilla):
				for i in range(0, x):
					dWA[i][j] = self.OUT
				j -= 1
	
		# Guardar las variables en las variables compartidas correspondiente
		self.lock.acquire()
		# Máximo y Mínimo posible en la ventana
		self.maxV, self.minV = self.VMAX - minVCasilla * self.VCHANGE, self.VMAX - maxVCasilla * self.VCHANGE
		self.minW, self.maxW = (minWCasilla -(self.X / 2)) * self.WCHANGE, (maxWCasilla - (self.X / 2)) * self.WCHANGE
		# Máximo y Mínimo posible en todo el rango de velocidades
		# self.maxV, self.minV = self.VMAX, 0
		# self.minW, self.maxW = self.WMIN, self.WMAX
		self.casillaVMax, self.casillaVMin = maxVCasilla, minVCasilla
		self.casillaWMax, self.casillaWMin = maxWCasilla, minWCasilla
		self.lock.release()


		t1 = time.clock_gettime(time.CLOCK_REALTIME)
		cont = 0

		# Se recorren todos los puntos
		for d in scan_sub:
			# Se guarda en una variable global si el valor tratado es el último
			self.LASTDISTANCE = cont == ( len(scan_sub) - 1)
			# Se calculan todos los pares de velocidades que son no admisibles
			pairVelocities = self.velocityAdmissible(d)

			# Se recorren todas las velocidades almacenadas en el array
			for velocity in pairVelocities:
				# Se comprueba que las velocidades están dentro del rango permitido
				if velocity[0] <= self.VMAX and velocity[0] >= self.VMIN and velocity[1] <= self.WMAX and velocity[1] >= self.WMIN:

					# Se calculan las casillas a las que corresponden las velocidades
					casillaW = (self.X / 2) + round(velocity[1] / self.WCHANGE)
					casillaV = (round(self.VMAX/self.VCHANGE)) - round(velocity[0] / self.VCHANGE)

					# Se actualiza el valor de dicha casilla, en la ventana
					if casillaV <= maxVCasilla and casillaV >= minVCasilla and casillaW <= maxWCasilla and casillaW >= minWCasilla:
						if casillaV < self.Y :
							# Se deben restar el mínimo de las velocidades para conseguir la casilla correspondiente en la ventana
							dWA[int(casillaV-minVCasilla)][int(casillaW-minWCasilla)] = self.FORBIDEN

					# Se actualiza el valor de dicha casilla, dentro de todo el rango
					# if casillaV < x and casillaV >= 0 and casillaW < y  and casillaW >= 0:
					# 	if casillaV < self.Y: 
					# 		dWA[int(casillaV)][int(casillaW)] = self.FORBIDEN
			cont += 1

				
		self.lock.acquire()
		self.matrix = dWA
		self.lock.release()
		t2 = time.clock_gettime(time.CLOCK_REALTIME) - t1
	
	
	def startPlot(self):
		'''
			Start the thread of the plot
		'''
		self.p = Process(target=self.plotDWA(), args=())
		self.p.start()

	def plotDWA(self):
		'''
			Plot the Space of Velocities 
		'''
		fig, ax = plt.subplots()
		minV, maxV, minW, maxW = 0.0 ,0.0, 0.0, 0.0
		# Colores asociados a los valores 0, 1, 2 o (FREE, FORBIDDEN, OUT)
		custom = colors.ListedColormap(["white", "red", "black"])
	

		while(True):
			t10 = time.clock_gettime(time.CLOCK_REALTIME)
			# Se actualizan los valores con las variables gloables
			self.lock.acquire()
			minV, maxV = self.minV, self.maxV
			minW, maxW = self.minW, self.maxW
			casillaVMin, casillaVMax = self.casillaVMin , self.casillaVMax 
			casillaWMin, casillaWMax = self.casillaWMin, self.casillaWMax 
			self.lock.release()

			# Se calcula el ancho, el centro en las X
			width = (casillaWMax - casillaWMin)
			centerX = casillaWMin + width/2
			
			# Se calcula la altura y el centro en las Y
			height = (casillaVMin - casillaVMax )
			centerY = casillaVMax + height/2
			
			# Etiquetas en los ejes de la ventana
			y = np.arange(maxV + self.VCHANGE, minV - self.VCHANGE, -self.VCHANGE)
			x = np.arange(minW - self.WCHANGE, maxW + self.WCHANGE, self.WCHANGE)

			# Etiquetas en los ejes de todo el rango de velocidades
			# y = np.arange(maxV + (maxV/8), minV - maxV/8, -maxV/8)
			# x = np.arange(minW - (2*maxW)/ 6, maxW + (2*maxW)/ 6, (2*maxW)/ 6)

			# Redondeo de las etiquetas
			x = np.round(x, 3)
			y = np.round(y, 2)

			ax.set_title("DWA")

			# Se muestra el espacio de velocidades
			self.lock.acquire()
			ax.matshow(self.matrix, cmap=custom, vmin = self.FREE, vmax= self.OUT)
			self.lock.release()

			# Se muestra la ventana de velocidades dinámicas, en el rango de velocidades
			# ax.add_patch(Rectangle((casillaWMin - 0.5, casillaVMax -0.5), width, height, edgecolor="green", facecolor='none', lw=1))
			
			# Se meustra la velocdiad actual
			# ax.add_patch(Rectangle((centerX - 0.75, centerY -0.75), 0.5, 0.5, edgecolor="blue", facecolor='none', lw=1))
			
			ax.set_xticklabels(x)
			ax.set_yticklabels(y)

			fig.show()
			plt.pause(0.001)
			ax.clear()

			# Se actualiza el valor para informar al hilo principal que ya está corriendo este hilo
			self.lock.acquire()
			self.wake = True
			self.lock.release()


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
		
		
		# Save the distances that the LaserScan catch
		scan_ranges = np.array(scan.ranges)

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
		self.ESQUINA = True
		self.INFERIOR = True
		self.DISTANCEBEFORE = 30


		middle = np.array(scan_ranges[478:600])
		top = np.array(scan_ranges[601:])
		bottom = np.array(scan_ranges[:477])
		div = 5

		self.DISTANCEANGLE = self.ANGLE_VISION/(int(len(scan_ranges)/div))


		th = np.arange(-self.ANGLE_VISION/2, self.ANGLE_VISION/2 + self.DISTANCEANGLE, self.DISTANCEANGLE)
		
		# Create a array with the minimum distance of the 10 positions and add the angle 
		scan_sub = np.array([[np.amin(scan_ranges[(k-div):k]), np.deg2rad(th[int((k/div) - 1)])] for k in np.arange(div, len(scan_ranges), div)])

	
		t6 = time.clock_gettime(time.CLOCK_REALTIME)
		TestNode.spaceVelocities(self, self.VBEF, self.WBEF, scan_sub)

		self.lock.acquire()
		posibilities = self.matrix
		self.lock.release()

		v, w = TestNode.searchVelocities(self, desired_v, desired_w, posibilities)

		self.VBEF = v
		self.WBEF = w

		# input("continue")
		self.send_vel(v,w)
		#rospy.loginfo(rospy.get_caller_id() + "Min range %f", np.minimum(scan_ranges))

def main():
    rospy.init_node("test_node", anonymous=True)

    test_node = TestNode()

    test_node.startPlot()
    
    rospy.spin() 
    
	

if __name__ == "__main__":
    main()
