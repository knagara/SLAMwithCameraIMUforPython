# -*- coding: utf-8 -*-

"""
sensor.py
Class for IMU sensors

methods:
	setData(data)
	processData()
	calcOrientation()
	calcRotationMatrix()
	calibrateAcceleration()
	calcGlobalAcceleration()
	init()
"""

import sys
from math import *
import cv2 as cv
import numpy as np

class Sensor:

	def __init__(self,_state):
		#state.py
		self.state = _state
		#variables
		self.accel = np.array([])
		self.gravity = np.array([])
		self.magnet = np.array([])
		self.magnet_fixed = np.array([])
		self.gyro = np.array([])
		self.orientation = np.array([0.0,0.0,0.0])
		self.cosx = 0.0
		self.cosy = 0.0
		self.cosz = 0.0
		self.offset1 = np.array([0.0,0.0,0.63776811])
		self.offset2 = np.array([0.0,0.0,1.15589317])
		self.offset_ = np.array([0.0,0.0,0.0])
		self.rotX = np.identity(3)
		self.rotY = np.identity(3)
		self.rotXY = np.identity(3)
		self.rot = np.identity(3)
		self.rotX_ = np.identity(3)
		self.rotY_ = np.identity(3)
		self.rotZ_ = np.identity(3)
		self.rot_ = np.identity(3)


	def init(self):
		#state.py
		self.state.init()
		#variables
		self.accel = np.array([])
		self.gravity = np.array([])
		self.magnet = np.array([])
		self.magnet_fixed = np.array([])
		self.gyro = np.array([])
		self.orientation = np.array([0.0,0.0,0.0])
		self.cosx = 0.0
		self.cosy = 0.0
		self.cosz = 0.0
		self.offset1 = np.array([0.0,0.0,0.63776811])
		self.offset2 = np.array([0.0,0.0,0.51812505])
		self.offset_ = np.array([0.0,0.0,0.0])
		self.rotX = np.identity(3)
		self.rotY = np.identity(3)
		self.rotXY = np.identity(3)
		self.rot = np.identity(3)
		self.rotX_ = np.identity(3)
		self.rotY_ = np.identity(3)
		self.rotZ_ = np.identity(3)
		self.rot_ = np.identity(3)


	#Set new data
	def setData(self,data):
		#set time
		self.state.setTime(float(long(data[0]) / 1000.0))

		#set sensor data
		self.accel = np.array([float(data[1]),float(data[2]),float(data[3])])
		self.gravity = np.array([-float(data[4]),-float(data[5]),-float(data[6])])
		self.magnet = np.array([float(data[7]),float(data[8]),float(data[9])])
		self.gyro = np.array([float(data[10]),float(data[11]),float(data[12])])


	#Execute all functions
	def processData(self):
		self.calcOrientation()
		self.calcRotationMatrix()
		#self.calibrateAcceleration()
		self.calcGlobalAcceleration()
		self.state.localization()


	#Calc orientation by using gravity and magnet
	#return orientation
	#see "Studies on Orientation Measurement in Sports Using Inertial and Magnetic Field Sensors"
	#    https://www.jstage.jst.go.jp/article/sposun/22/2/22_255/_pdf
	def calcOrientation(self):
		#x roll
		self.orientation[0] = atan2(self.gravity[1],self.gravity[2])
		#y pitch
		#sign(+ or -) is decided here
		sign = 1.0
		if(self.gravity[2]<0): #decided by z axis
			sign = -1.0
		self.orientation[1] = atan2(-self.gravity[0],sign*hypot(self.gravity[1],self.gravity[2]))
		#z yaw
		cv.Rodrigues(np.array((self.orientation[0],0.0,0.0)),self.rotX)
		cv.Rodrigues(np.array((0.0,self.orientation[1],0.0)),self.rotY)
		self.rotXY = np.dot(self.rotY,self.rotX)
		self.magnet_fixed = np.dot(self.rotXY,self.magnet)
		self.orientation[2] = atan2(-self.magnet_fixed[1],self.magnet_fixed[0])

		self.state.setOrientation(self.orientation)

		print(str(degrees(self.orientation[0]))+" "+str(degrees(self.orientation[1]))+" "+str(degrees(self.orientation[2])))


	#Calc rotation matrix from orientation
	def calcRotationMatrix(self):
		#Rotation matrix R(Z)R(Y)R(X)
		cv.Rodrigues(np.array((self.orientation[0],0.0,0.0)),self.rotX_)
		cv.Rodrigues(np.array((0.0,self.orientation[1],0.0)),self.rotY_)
		cv.Rodrigues(np.array((0.0,0.0,self.orientation[2])),self.rotZ_)
		self.rot_ = np.dot(self.rotZ_,np.dot(self.rotY_,self.rotX_))


	#Calibrate acceleration
	def calibrateAcceleration(self):
		#angle between gravity and device
		self.cosx = self.rot_[0,0]
		self.cosy = self.rot_[1,1]
		self.cosz = self.rot_[2,2]

		#Calc offset
		if(self.cosz<0):
			self.offset_[2] = -(self.offset1[2] * self.cosz)
		else:
			self.offset_[2] = self.offset2[2] * self.cosz

		#Add offset
		self.accel = self.accel + self.offset_


	#Calc accel in global coordinates by using orientation
	def calcGlobalAcceleration(self):
		#global accel = R(Z)R(Y)R(X) * accel
		self.state.setAccel(np.dot(self.rot_,self.accel))

		#print(self.a)
		#print(self.accel)
		#print(self.a+np.dot(self.rot_,self.gravity))
		#print(np.dot(self.rot_,self.gravity))

