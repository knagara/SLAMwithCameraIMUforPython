# -*- coding: utf-8 -*-

"""
state_RBPF.py

author: Keita Nagara (University of Tokyo) <nagara.keita()gmail.com>

State and estimation model of Rao-Blackwellized Particle Filter.

This class is generated from "state.py".

"""

import sys
import math
import time
import cv2 as cv
import numpy as np
from particle_filter import ParticleFilter
from particle import Particle

class StateRBPF:

	def __init__(self):
		# ----- Set parameters here! ----- #
		self.M = 100 # total number of particles パーティクルの数
		self.f = 1575.54144 # focus length of camera [px] カメラの焦点距離 [px]
		# Particle Filter
		self.noise_a_sys = 0.01 # system noise of acceleration　加速度のシステムノイズ
		self.noise_g_sys = 0.01 # system noise of gyro　ジャイロのシステムノイズ
		self.noise_camera = 0.1 # observation noise of camera カメラの観測ノイズ
		# ----- Set parameters here! ----- #

		self.init()

	def init(self):
		self.isFirstTimeIMU = True
		self.isFirstTimeCamera = True
		self.lock = False

		self.t = 0.0
		self.t1 = 0.0

		self.initParticleFilter()


	def initParticleFilter(self):
		self.pf = ParticleFilter().getParticleFilterClass("RBPF")
		self.pf.setFocus(self.f)
		self.pf.setParameter(self.noise_a_sys, self.noise_g_sys, self.noise_camera) #パーティクルフィルタのパラメータ（ノイズ） parameters (noise)
		self.X = [] # パーティクルセット set of particles
		self.loglikelihood = 0.0
		self.count = 1
		self.step = 1


	def initParticle(self, accel, ori):
		X = []
		for i in xrange(self.M):
			particle = Particle()
			particle.initWithIMU(accel, ori)
			X.append(particle)
		return X



	"""
	This method is called from "sensor.py" when new IMU sensor data are arrived.
	time : time (sec)
	accel : acceleration in global coordinates
	ori : orientaion
	"""
	def setSensorData(self, time_, accel, ori):

		# If process is locked by Image Particle Filter, do nothing
		if(self.lock):
			print("locked")
			return

		# Get current time
		self.t1 = self.t
		self.t = time_
		self.dt = self.t - self.t1

		if(self.isFirstTimeIMU):
			# init particle
			self.X = self.initParticle(accel, ori)
		else:
			# exec particle filter
			self.X = self.pf.pf_step_IMU(self.X, self.dt, accel, ori, self.M)
			""" The code below is used to get loglikelihood to decide parameters.
			self.X, likelihood = self.pf.pf_step_IMU(self.X, self.t - self.t1, accel, ori, self.M)
			self.loglikelihood += math.log(likelihood)
			self.count += 1
			if(self.count==300):
				print(str(self.loglikelihood))
			"""

		if(self.isFirstTimeIMU):
			self.isFirstTimeIMU = False

		# Count
		self.count+=1


	"""
	This method is called from Image class when new camera image data are arrived.
	time_ : time (sec)
	keypointPairs : list of KeyPointPair class objects
	"""
	def setImageData(self, time_, keypoints):

		# If IMU data has not been arrived yet, do nothing
		if(self.isFirstTimeIMU):
			return

		if(self.isFirstTimeCamera):
			self.isFirstTimeCamera = False

		# Lock IMU process
		self.lock = True

		# Get current time
		self.t1 = self.t
		self.t = time_
		self.dt = self.t - self.t1
		
		# covariance matrix of position
		P = self.createPositionCovarianceMatrixFromParticle(self.X)
		
		# exec particle filter
		self.X = self.pf.pf_step_camera(self.X, self.dt, keypoints, self.step, P, self.M)

		# Count
		self.count+=1
		
		# Step (camera only observation step)
		self.step += 1
		
		# Unlock IMU process
		self.lock = False



	"""
	create covariance matrix of position from particle set
	"""
	def createPositionCovarianceMatrixFromParticle(self, X):
		x = []
		for X_ in X:
			if(len(x)==0):
				x = X_.x
			else:
				x = np.vstack((x,X_.x))
		P = np.cov(x.T)
		return P



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
		#print(np.var(x, axis=0))
		return np.mean(x, axis=0),np.mean(v, axis=0),np.mean(a, axis=0),np.mean(o, axis=0)

