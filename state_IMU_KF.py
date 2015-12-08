# -*- coding: utf-8 -*-

"""
state_IMU_KF.py

author: Keita Nagara　永良慶太 (University of Tokyo) <nagara.keita()gmail.com>

State and estimation model of IMU with Kalman Filter.

This class is generated from "state.py".

"""

import sys
import math
import cv2 as cv
import numpy as np
import KF

class StateIMUKF:

	def __init__(self):
		self.init()


	def init(self):
		self.isFirstTime = True
		self.t = 0
		self.t1 = 0
		
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
		self.Q = np.diag([0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]) # sys noise
		self.R = np.diag([0.01,0.01,0.01,0.01,0.01,0.01]) # obs noise
		


	"""
	This method is called from "sensor.py" when new IMU sensor data are arrived.
	time : time (sec)
	accel : acceleration in global coordinates
	ori : orientaion
	"""
	def setSensorData(self, time, accel, ori):

		self.t1 = self.t
		self.t = time

		if(self.isFirstTime):
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
			self.mu, self.sigma = KF.execKF1Simple(Y,self.mu,self.sigma,self.A,self.C,self.Q,self.R)

		if(self.isFirstTime):
			self.isFirstTime = False

		
	
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

