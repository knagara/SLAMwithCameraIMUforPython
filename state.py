# -*- coding: utf-8 -*-

"""
state.py

author: Keita Nagara (University of Tokyo)

This class is called from "sensor.py" and "image.py", and estimate state variables using particle filter.

"""

import sys
import math
import cv2 as cv
import numpy as np
import KF
from particle_filter import ParticleFilter
from particle import Particle


class State:

	def __init__(self):
		self.init()


	def init(self):
		self.isFirstTime = True
		self.t = 0
		self.t1 = 0
		
		self.initKalmanFilter()
		#self.initParticleFilter()


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
		#self.Q = np.diag([0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]) # sys noise
		#self.R = np.diag([0.01,0.01,0.01,0.01,0.01,0.01]) # obs noise
		
		
	def initParticleFilter(self):
		self.pf = ParticleFilter()
		self.pf.setParameter(math.pow(2,-8) , math.pow(10,-4)) #パーティクルフィルタのパラメータ（ノイズの分散）
		self.M = 100 # パーティクルの数 num of particles
		self.X = [] # パーティクルセット set of particles
		self.loglikelihood = 0.0
		self.count = 0
		


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
			self.Q = np.diag([0.0001,0.0001,0.0001,0.0001,0.0001,0.0001,0.1,0.1,0.1,0.05,0.05,0.05]) # sys noise
			self.R = np.diag([0.01,0.01,0.01,0.005,0.005,0.005]) # obs noise
			self.mu, self.sigma = KF.execKF1Simple(Y,self.mu,self.sigma,self.A,self.C,self.Q,self.R)
			

		if(self.isFirstTime):
			self.isFirstTime = False


	"""
	This method is called from "sensor.py" when new IMU sensor data are arrived.
	time : time (sec)
	accel : acceleration in global coordinates
	ori : orientaion
	"""
	def setSensorDataPF(self, time, accel, ori):

		self.t1 = self.t
		self.t = time

		if(self.isFirstTime):
			# init particle
			self.X = self.initParticle(accel, ori)
		else:
			# exec particle filter
			self.X, likelihood = self.pf.pf_step_IMU(self.X, self.t - self.t1, accel, ori, self.M)
			self.loglikelihood += math.log(likelihood)
			self.count += 1
			if(self.count==300):
				print(str(self.loglikelihood))	

		if(self.isFirstTime):
			self.isFirstTime = False


	def initParticle(self, accel, ori):
		X = []
		particle = Particle(accel, ori)
		for i in range(self.M):
			X.append(particle)
		return X
		
	
	def getState(self):
		x = np.array([self.mu[0],self.mu[1],self.mu[2]])
		v = np.array([self.mu[3],self.mu[4],self.mu[5]])
		a = np.array([self.mu[6],self.mu[7],self.mu[8]])
		o = np.array([self.mu[9],self.mu[10],self.mu[11]])
		return x,v,a,o


	def getStatePF(self):
		x = []
		v = []
		a = []
		o = []
		for X_ in self.X:
			x.append(X_.x)
			v.append(X_.v)
			a.append(X_.a)
			o.append(X_.o)
		return np.mean(x, axis=0),np.mean(v, axis=0),np.mean(a, axis=0),np.mean(o, axis=0)



