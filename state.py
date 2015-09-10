# -*- coding: utf-8 -*-

"""
state.py

author: Keita Nagara (University of Tokyo)

This class is called from "sensor.py" and "image.py", and estimate state variables using particle filter.

"""

import sys
from math import *
import cv2 as cv
import numpy as np
import KF
from particle_filter import ParticleFilter
from particle import Particle


class State:

	def __init__(self):
		self.init()


	def init(self):
		self.pf = ParticleFilter()
		self.isFirstTime = True
		self.M = 100 # パーティクルの数 num of particles
		self.X = [] # パーティクルセット set of particles
		self.t = 0
		self.t1 = 0


	"""
	This method is called from "sensor.py"
	"""
	def setSensorData(self, time, accel, ori):

		self.t1 = self.t
		self.t = time

		if(self.isFirstTime):
			# init particle
			self.X = self.initParticle(accel, ori)
		else:
			# exec particle filter
			self.X = self.pf.pf_step_IMU(self.X, self.t - self.t1, accel, ori, self.M)

		if(self.isFirstTime):
			self.isFirstTime = False


	def initParticle(self, accel, ori):
		X = []
		particle = Particle(accel, ori)
		for i in range(self.M):
			X.append(particle)
		return X


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



