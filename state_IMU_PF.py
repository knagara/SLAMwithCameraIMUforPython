# -*- coding: utf-8 -*-

"""
state_IMU_PF.py

author: Keita Nagara (University of Tokyo)

State and estimation model of IMU with Particle Filter.

This class is generated from "state.py".

"""

import sys
import math
import cv2 as cv
import numpy as np
from particle_filter import ParticleFilter
from particle import Particle

class StateIMUPF:

	def __init__(self):
		pass

	def initWithType(self,PFtype_):
		self.PFtype = PFtype_
		self.init()

	def init(self):
		self.isFirstTime = True
		self.t = 0.0
		self.t1 = 0.0
		
		self.initParticleFilter(self.PFtype)
		
		
	def initParticleFilter(self,PFtype):
		self.pf = ParticleFilter().getParticleFilterClass(PFtype) #PFtype = "IMUPF" or "IMUPF2"
		self.pf.setParameter(math.pow(10,-10) , math.pow(10,-10)) #パーティクルフィルタのパラメータ（ノイズの分散） variance of noise
		self.M = 100 # パーティクルの数 num of particles
		self.X = [] # パーティクルセット set of particles
		self.loglikelihood = 0.0
		self.count = 0


	def initParticle(self, accel, ori):
		X = []
		particle = Particle(accel, ori)
		for i in range(self.M):
			X.append(particle)
		return X
		


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
			# init particle
			self.X = self.initParticle(accel, ori)
		else:
			# exec particle filter
			self.X = self.pf.pf_step(self.X, self.t - self.t1, accel, ori, self.M)
			""" The code below is used to get loglikelihood to decide parameters.
			self.X, likelihood = self.pf.pf_step(self.X, self.t - self.t1, accel, ori, self.M)
			self.loglikelihood += math.log(likelihood)
			self.count += 1
			if(self.count==300):
				print(str(self.loglikelihood))	
			"""

		if(self.isFirstTime):
			self.isFirstTime = False


	"""
	This method is called from "Main.py"
	return estimated state vector
	"""
	def getState(self):
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

