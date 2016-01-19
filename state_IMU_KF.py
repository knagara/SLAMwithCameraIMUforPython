# -*- coding: utf-8 -*-

"""
state_IMU_KF.py

author: Keita Nagara　永良慶太 (University of Tokyo) <nagara.keita()gmail.com>

State and estimation model of IMU with Kalman Filter.

This class is generated from "state.py".

"""

import sys
import math
import time
import copy
import cv2 as cv
import numpy as np
import KF
import Util

class StateIMUKF:

	def __init__(self):
		self.init()


	def init(self):
		self.isFirstTime = True
		self.t = 0
		self.t1 = 0
		self.accel1 = np.array([0.0, 0.0, 0.0])
		self.accel2 = np.array([0.0, 0.0, 0.0])
		self.accel3 = np.array([0.0, 0.0, 0.0])
		
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
	def setSensorData(self, time_, accel, ori, gyro):

		self.t1 = self.t
		self.t = time_

		if(self.isFirstTime):
			#init mu
			self.mu = np.array([0.0,0.0,0.0,
							0.0,0.0,0.0,
							accel[0],accel[1],accel[2],
							ori[0],ori[1],ori[2]])
		else:
			
			#start_time = time.clock()
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
				
			"""
			self.accel3 = copy.deepcopy(self.accel2)
			self.accel2 = copy.deepcopy(self.accel1)
			self.accel1 = copy.deepcopy(accel)
			if(Util.isDeviceMoving(self.accel1[0]) == False and Util.isDeviceMoving(self.accel2[0]) == False and Util.isDeviceMoving(self.accel3[0]) == False):
				self.mu[3] = 0.0
			if(Util.isDeviceMoving(self.accel1[1]) == False and Util.isDeviceMoving(self.accel2[1]) == False and Util.isDeviceMoving(self.accel3[1]) == False):
				self.mu[4] = 0.0
			if(Util.isDeviceMoving(self.accel1[2]) == False and Util.isDeviceMoving(self.accel2[2]) == False and Util.isDeviceMoving(self.accel3[2]) == False):
				self.mu[5] = 0.0
			"""
			self.mu, self.sigma = KF.execKF1Simple(Y,self.mu,self.sigma,self.A,self.C,self.Q,self.R)

			#end_time = time.clock()
			#print "%f" %(end_time-start_time)
		
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

