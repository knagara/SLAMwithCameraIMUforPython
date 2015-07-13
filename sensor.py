# -*- coding: utf-8 -*-

"""
sensor.py
Class for IMU sensors

methods:
	setData(data)
	processData()
	calcOrientation()
	calcRotationMatrix()
	calcGlobalAcceleration()
	init()
"""

import sys
from math import *
import cv2 as cv
import numpy as np
import Util

class Sensor:

	def __init__(self,_state):
		#state.py
		self.state = _state
		#variables
		self.isFirstTime = True
		self.accel = np.array([])
		self.gravity = np.array([])
		self.magnet = np.array([])
		self.magnet_fixed = np.array([])
		self.gyro = np.array([])
		self.orientation = np.array([0.0,0.0,0.0])
		self.orientation_g = np.array([0.0,0.0,0.0])
		self.orientation_gyro = np.array([0.0,0.0,0.0])
		self.cosx = 0.0
		self.cosy = 0.0
		self.cosz = 0.0
		self.rotX = np.identity(3)
		self.rotY = np.identity(3)
		self.rotXY = np.identity(3)
		self.rot = np.identity(3)
		self.rotX_ = np.identity(3)
		self.rotY_ = np.identity(3)
		self.rotZ_ = np.identity(3)
		self.rot_ = np.identity(3)
		self.gyro1 = np.array([])
		self.r = np.array([0.0,0.03,0.0])


	def init(self):
		#state.py
		self.state.init()
		#variables
		self.isFirstTime = True
		self.accel = np.array([])
		self.gravity = np.array([])
		self.magnet = np.array([])
		self.magnet_fixed = np.array([])
		self.gyro = np.array([])
		self.orientation = np.array([0.0,0.0,0.0])
		self.orientation_g = np.array([0.0,0.0,0.0])
		self.orientation_gyro = np.array([0.0,0.0,0.0])
		self.cosx = 0.0
		self.cosy = 0.0
		self.cosz = 0.0
		self.rotX = np.identity(3)
		self.rotY = np.identity(3)
		self.rotXY = np.identity(3)
		self.rot = np.identity(3)
		self.rotX_ = np.identity(3)
		self.rotY_ = np.identity(3)
		self.rotZ_ = np.identity(3)
		self.rot_ = np.identity(3)
		self.gyro1 = np.array([])
		self.r = np.array([0.0,0.03,0.0])



	#Set new data and Execute all functions
	def processData(self,data):

		if(self.isFirstTime==False):
			self.state.setTime(float(long(data[0]) / 1000.0))
			self.gyro1 = self.gyro
			
		self.accel = np.array([float(data[1]),float(data[2]),float(data[3])])
		self.gravity = np.array([-float(data[4]),-float(data[5]),-float(data[6])])
		self.magnet = np.array([float(data[7]),float(data[8]),float(data[9])])
		self.gyro = np.array([float(data[10])+0.016076385,float(data[11])-0.002585781,float(data[12])-0.019482931]) #add offset
			
		self.calcOrientation()
		self.calcRotationMatrix()
		
		if(self.isFirstTime==False):
			self.removeCentrifugalAndTangentialAccel()
			self.calcGlobalAcceleration()
			self.state.localization()
			
		if(self.isFirstTime):
			self.isFirstTime = False


	#Calc orientation
	def calcOrientation(self):
		self.calcOrientationByGravity()
		self.calcOrientationByGyro()

		self.orientation = self.orientation_g

		#set orientation to state class
		self.state.setOrientation(self.orientation)


	#Calc orientation by using gyro
	def calcOrientationByGyro(self):
		if(self.isFirstTime):
			self.orientation_gyro = self.orientation_g
		else:
			t = self.state.getTimeDelta()
			self.orientation_gyro = self.orientation_gyro + self.gyro * t
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
		#y pitch
		self.orientation_g[1] = atan2(-self.gravity[0],hypot(self.gravity[1],self.gravity[2]))
		#sign(+ or -) is decided here
		#sign = 1.0
		#if(self.gravity[2]<0): #decided by z axis
		#	sign = -1.0
		#self.orientation_g[1] = atan2(-self.gravity[0],sign*hypot(self.gravity[1],self.gravity[2]))
		#z yaw
		self.rotX = Util.rotationMatrixX(self.orientation_g[0])
		self.rotY = Util.rotationMatrixY(self.orientation_g[1])
		self.rotXY = np.dot(self.rotY,self.rotX)
		self.magnet_fixed = np.dot(self.rotXY,self.magnet)
		self.orientation_g[2] = atan2(-self.magnet_fixed[1],self.magnet_fixed[0])


	#Calc rotation matrix from orientation
	def calcRotationMatrix(self):
		#Rotation matrix R(Z)R(Y)R(X)
		self.rotX_ = Util.rotationMatrixX(self.orientation[0])
		self.rotY_ = Util.rotationMatrixY(self.orientation[1])
		self.rotZ_ = Util.rotationMatrixZ(self.orientation[2])
		self.rot_ = np.dot(self.rotZ_,np.dot(self.rotY_,self.rotX_))
		
		
	#Remove Centrifugal and Tangential Accel
	#see also "Studies on Orientation Measurement in Sports Using Inertial and Magnetic Field Sensors"
	#         https://www.jstage.jst.go.jp/article/sposun/22/2/22_255/_pdf
	def removeCentrifugalAndTangentialAccel(self):
		#Angular velocity
		wv = self.gyro
		#Angular acceleration
		wa = (self.gyro - self.gyro1)/self.state.getTimeDelta()
		
		# a = a - wv*(wv*r) - wa*r
		self.accel = self.accel - np.cross(wv,np.cross(wv,self.r)) - np.cross(wa,self.r)
		
		

	#Calc accel in global coordinates by using orientation
	def calcGlobalAcceleration(self):
		#accel in global = R(Z)R(Y)R(X) * accel
		self.state.setAccel(np.dot(self.rot_,self.accel))

