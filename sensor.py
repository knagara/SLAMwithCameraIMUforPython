# -*- coding: utf-8 -*-

"""
sensor.py

author: Keita Nagara　永良慶太 (University of Tokyo) <nagara.keita()gmail.com>

This class is called from "Main.py", and process sensor data.

"""

from math import *
import numpy as np
import Util
import KF

class Sensor:

	def __init__(self,_state):
		#state.py
		self.state = _state
		self.init()


	def init(self):
		#variables
		self.isFirstTime = True
		self.time = 0.0
		self.time1 = 0.0
		self.accel = np.array([])
		self.accel_g = np.array([])
		self.gravity = np.array([]) #
		self.magnet = np.array([])
		self.magnet_fixed = np.array([])
		self.gyro = np.array([])
		self.gyro_diff = np.array([])
		self.orientation = np.array([0.0,0.0,0.0])
		self.orientation_g = np.array([0.0,0.0,0.0])
		self.orientation_gyro = np.array([0.0,0.0,0.0])
		self.rotX_ = np.identity(3)
		self.rotY_ = np.identity(3)
		self.rotX = np.identity(3)
		self.rotY = np.identity(3)
		self.rotZ = np.identity(3)
		self.rot = np.identity(3)
		self.I = np.identity(3)
		self.P = np.array([0.0,0.0,0.0]) # covariance matrix of KF for orientation
		self.Q = np.diag([0.1,0.1,0.1]) # noise of KF for orientation
		self.R = np.diag([0.01,0.01,0.01]) # noise of KF for orientation
		self.centrifugal = np.array([0.0,0.0,0.0]) #
		self.tangential = np.array([0.0,0.0,0.0]) #
		self.r = np.array([0.0,0.0,0.0])
		self.v = np.array([0.0,0.0,0.0])
		self.v1 = np.array([0.0,0.0,0.0])



	#Set new data and Execute all functions
	def processData(self,data):

		self.time1 = self.time
		self.time = (float(long(data[0]) / 1000.0))

		self.accel = np.array([float(data[1]),float(data[2]),float(data[3])])
		self.orientation_g = np.array([float(data[4]),float(data[5]),float(data[6])])
		self.magnet = np.array([float(data[7]),float(data[8]),float(data[9])])
		self.gyro = np.array([float(data[10]),float(data[11]),float(data[12])])
		#self.gyro_diff = np.array([float(data[13]),float(data[14]),float(data[15])])

		#self.calcOrientation()
		#self.calcRotationMatrix()
		#self.calcGlobalAcceleration()
		self.pushDataToState()

		if(self.isFirstTime):
			self.isFirstTime = False


	#Calc orientation
	def calcOrientation(self):
		self.calcOrientationByGravity()
		if(self.isFirstTime):
			self.orientation = self.orientation_g
		else:
			t = self.time - self.time1
			matrixGyro2Euler = Util.matrixGyro2Euler(self.orientation[0],self.orientation[1]) * t

			#Kalman Filter
			resultKF = KF.execKF1(self.orientation_g, self.gyro, self.orientation, self.P, self.I, matrixGyro2Euler, self.I, self.Q, self.R)
			self.orientation = resultKF[0]
			self.P = resultKF[1]

			if(self.orientation[0]>=pi):
				self.orientation[0] -= pi*2.0
			if(self.orientation[1]>=pi):
				self.orientation[1] -= pi*2.0
			if(self.orientation[2]>=pi):
				self.orientation[2] -= pi*2.0
			if(self.orientation[0]<-pi):
				self.orientation[0] += pi*2.0
			if(self.orientation[1]<-pi):
				self.orientation[1] += pi*2.0
			if(self.orientation[2]<-pi):
				self.orientation[2] += pi*2.0



	#Calc orientation by using gyro
	def calcOrientationByGyro(self):
		if(self.isFirstTime):
			self.orientation_gyro = self.orientation_g
		else:
			t = self.time - self.time1
			gyroEuler = np.dot(Util.matrixGyro2Euler(self.orientation_gyro[0],self.orientation_gyro[1]),self.gyro)
			self.orientation_gyro = self.orientation_gyro + gyroEuler * t

			if(self.orientation_gyro[0]>=pi):
				self.orientation_gyro[0] -= pi*2.0
			if(self.orientation_gyro[1]>=pi):
				self.orientation_gyro[1] -= pi*2.0
			if(self.orientation_gyro[2]>=pi):
				self.orientation_gyro[2] -= pi*2.0
			if(self.orientation_gyro[0]<-pi):
				self.orientation_gyro[0] += pi*2.0
			if(self.orientation_gyro[1]<-pi):
				self.orientation_gyro[1] += pi*2.0
			if(self.orientation_gyro[2]<-pi):
				self.orientation_gyro[2] += pi*2.0


	#Calc orientation by using gravity and magnet
	#return orientation
	#see also "Studies on Orientation Measurement in Sports Using Inertial and Magnetic Field Sensors"
	#         https://www.jstage.jst.go.jp/article/sposun/22/2/22_255/_pdf
	def calcOrientationByGravity(self):
		#x roll
		self.orientation_g[0] = atan2(self.gravity[1],self.gravity[2])
		#y pitch (-90 ～ +90)
		self.orientation_g[1] = atan2(-self.gravity[0],hypot(self.gravity[1],self.gravity[2]))
		#y pitch (-180 ～ +180)
		"""
		if(self.gravity[2]<0): #decided by z axis
			self.orientation_g[1] = atan2(-self.gravity[0],-hypot(self.gravity[1],self.gravity[2]))
		else:
			self.orientation_g[1] = atan2(-self.gravity[0],hypot(self.gravity[1],self.gravity[2]))
		"""
		#z yaw
		self.rotX_ = Util.rotationMatrixX(self.orientation_g[0])
		self.rotY_ = Util.rotationMatrixY(self.orientation_g[1])
		self.magnet_fixed = np.dot(np.dot(self.rotY_,self.rotX_),self.magnet)
		self.orientation_g[2] = atan2(-self.magnet_fixed[1],self.magnet_fixed[0])


	#Calc rotation matrix from orientation
	def calcRotationMatrix(self):
		#Rotation matrix R(Z)R(Y)R(X)
		self.rotX = Util.rotationMatrixX(self.orientation[0])
		self.rotY = Util.rotationMatrixY(self.orientation[1])
		self.rotZ = Util.rotationMatrixZ(self.orientation[2])
		self.rot = np.dot(self.rotZ,np.dot(self.rotY,self.rotX))


	#Remove Centrifugal Accel
	def removeCentrifugalAndTangentialAccel(self):
		#Angular velocity
		w = self.gyro
		#Angular acceleration
		#wa = self.gyro_diff
		#wn2 (norm of gyro vector)^2
		wn2 = pow(np.linalg.norm(w),2)
		#norm of global v
		vn = np.linalg.norm(self.state.v)
		#centrifugal
		centrifugal_x = np.array([0.0,0.0,0.0])
		centrifugal_y = np.array([0.0,0.0,0.0])
		if(w[0] > 0.3 or w[0] < -0.3):
			#v of x
			vx = np.array([0.0,vn*sin(self.orientation[0]),vn*cos(self.orientation[0])])
			#w of x
			wx = np.array([w[0],0.0,0.0])
			#w*(w*r) of x
			centrifugal_x = np.cross(wx,np.cross(wx,np.cross(vx,wx)/wn2))
		if(w[1] > 0.3 or w[1] < -0.3):
			#v of y
			vy = np.array([-vn*sin(self.orientation[1]),0.0,vn*cos(self.orientation[1])])
			#w of y
			wy = np.array([0.0,w[1],0.0])
			#w*(w*r) of y
			centrifugal_y = np.cross(wy,np.cross(wy,np.cross(vy,wy)/wn2))
		#a = a - w*(w*r)
		self.centrifugal = centrifugal_x + centrifugal_y
		self.accel = self.accel - self.centrifugal



	#Calc accel in global coordinates by using orientation
	def calcGlobalAcceleration(self):
		#accel in global = R(Z)R(Y)R(X) * accel
		self.accel_g = np.dot(self.rot,self.accel)


	#Push all data to State class
	def pushDataToState(self):
		self.state.setSensorData(self.time, self.accel, self.orientation_g)

