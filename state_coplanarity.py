# -*- coding: utf-8 -*-

"""
state_coplanarity.py

author: Keita Nagara (University of Tokyo)

State and estimation model of Coplanarity (IMU with Kalman Filter & Camera with Particle Filter. Observation model is coplanarity. State vector is device state only).

This class is generated from "state.py".

"""

import sys
import time
import math
import cv2 as cv
import numpy as np
import KF

class StateCoplanarity:

	def __init__(self):
		self.init()


	def init(self):
		self.isFirstTimeIMU = True
		self.lock = False
		
		self.t = 0.0
		self.t1 = 0.0
		
		self.initKalmanFilter()


	def initKalmanFilter(self):
		self.mu = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
		self.sigma = np.zeros([12,12])
		self.A = np.identity(12)
		self.C = np.array([		[0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0],
							[0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0],
							[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0],
							[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0],
							[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0],
							[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0]])
		self.Q = np.diag([0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.01,0.01,0.01]) # sys noise
		self.R = np.diag([0.001,0.001,0.001,0.000001,0.000001,0.000001]) # obs noise
		


	"""
	This method is called from Sensor class when new IMU sensor data are arrived.
	time_ : time (sec), but never used
	accel : acceleration in global coordinates
	ori : orientaion
	"""
	def setSensorData(self, time_, accel, ori):
		
		# if process is locked by Image Particle Filter, do nothing
		if(self.lock):
			print("locked")
			return

		self.t1 = self.t
		self.t = time.time()

		if(self.isFirstTimeIMU):
			#init mu
			self.mu = np.array([0.0,0.0,0.0,
							0.0,0.0,0.0,
							accel[0],accel[1],accel[2],
							ori[0],ori[1],ori[2]])
		else:
			#observation
			Y = np.array([accel[0],accel[1],accel[2],
						ori[0],ori[1],ori[2]])
			dt = self.t - self.t1
			dt2 = 0.5 * dt * dt
			dt3 = (1.0 / 6.0) * dt2 * dt
			self.A = np.array([[1.0,0.0,0.0,dt,0.0,0.0,dt2,0.0,0.0,0.0,0.0,0.0],
							[0.0,1.0,0.0,0.0,dt,0.0,0.0,dt2,0.0,0.0,0.0,0.0],
							[0.0,0.0,1.0,0.0,0.0,dt,0.0,0.0,dt2,0.0,0.0,0.0],
							[0.0,0.0,0.0,1.0,0.0,0.0,dt,0.0,0.0,0.0,0.0,0.0],
							[0.0,0.0,0.0,0.0,1.0,0.0,0.0,dt,0.0,0.0,0.0,0.0],
							[0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,dt,0.0,0.0,0.0],
							[0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0],
							[0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0],
							[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0],
							[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0],
							[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0],
							[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0]])
			Qt = np.diag([dt3,dt3,dt3,dt2,dt2,dt2,dt,dt,dt,dt2,dt2,dt2])
			Q = Qt.dot(self.Q)
			self.mu, self.sigma = KF.execKF1Simple(Y,self.mu,self.sigma,self.A,self.C,Q,self.R)

		if(self.isFirstTimeIMU):
			self.isFirstTimeIMU = False



	"""
	This method is called from Image class when new camera image data are arrived.
	keypoints : list of KeyPointPair class objects
	"""
	def setImageData(self, keypoints):
		
		# Lock IMU process
		self.lock = True
		
		self.t1 = self.t
		self.t = time.time()
		
		# Unlock IMU process
		self.lock = False
		
	
	"""
	This method is called from "Main.py"
	return estimated state vector
	"""
	def getState(self):
		x = np.array([self.mu[0],self.mu[1],self.mu[2]])
		v = np.array([self.mu[3],self.mu[4],self.mu[5]])
		a = np.array([self.mu[6],self.mu[7],self.mu[8]])
		o = np.array([self.mu[9],self.mu[10],self.mu[11]])
		return x,v,a,o

