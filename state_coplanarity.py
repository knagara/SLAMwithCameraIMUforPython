# -*- coding: utf-8 -*-

"""
state_coplanarity.py

author: Keita Nagara (University of Tokyo)

State and estimation model of Coplanarity (IMU with Kalman Filter & Camera with Particle Filter. Observation model is coplanarity. State vector is device state only).

This class is generated from "state.py".

"""

import sys
import time
import copy
import math
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D
import KF
from particle_filter import ParticleFilter
from particle import Particle

class StateCoplanarity:

	def __init__(self):
		
		# ----- Set parameters here! ----- #
		self.M = 100 # total number of particles パーティクルの数
		self.f = 1575.54144 # focus length of camera [px] カメラの焦点距離 [px]
		# Kalman Filter
		self.noise_a_sys = 0.01 # system noise of acceleration　加速度のシステムノイズ
		self.noise_g_sys = 0.01 # system noise of gyro　ジャイロのシステムノイズ
		self.noise_a_obs = 0.00000001 # observation noise of acceleration　加速度の観測ノイズ
		self.noise_g_obs = 0.00000001 # observation noise of gyro　ジャイロの観測ノイズ
		# Particle Filter
		self.PFnoise_a_sys = 10.0 # system noise of acceleration　加速度のシステムノイズ
		self.PFnoise_g_sys = 10.0 # system noise of gyro　ジャイロのシステムノイズ
		self.PFnoise_a_obs = 0.00000001 # observation noise of acceleration　加速度の観測ノイズ
		self.PFnoise_g_obs = 0.00000001 # observation noise of gyro　ジャイロの観測ノイズ
		self.PFnoise_coplanarity_obs = 0.1 # observation noise of coplanarity 共面条件の観測ノイズ
		# ----- Set parameters here! ----- #
		
		self.init()


	def init(self):
		self.isFirstTimeIMU = True
		self.isFirstTimeCamera = True
		self.lock = False
		
		self.t = 0.0
		self.t1 = 0.0
		
		self.initKalmanFilter()
		self.initParticleFilter()


	def initKalmanFilter(self):
		self.mu = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
		self.mu1 = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
		self.sigma = np.zeros([12,12])
		self.C = np.array([		[0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0],
							[0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0],
							[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0],
							[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0],
							[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0],
							[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0]])
		self.Q = np.diag([self.noise_a_sys,self.noise_a_sys,self.noise_a_sys,self.noise_a_sys,self.noise_a_sys,self.noise_a_sys,self.noise_a_sys,self.noise_a_sys,self.noise_a_sys,self.noise_g_sys,self.noise_g_sys,self.noise_g_sys]) # sys noise
		self.R = np.diag([self.noise_a_obs,self.noise_a_obs,self.noise_a_obs,self.noise_g_obs,self.noise_g_obs,self.noise_g_obs]) # obs noise
		

	def initParticleFilter(self):
		self.pf = ParticleFilter().getParticleFilterClass("Coplanarity")
		self.pf.setFocus(self.f)
		self.pf.setParameter(self.PFnoise_a_sys, self.PFnoise_g_sys, self.PFnoise_a_obs, self.PFnoise_g_obs, self.PFnoise_coplanarity_obs) #パーティクルフィルタのパラメータ（ノイズ） parameters (noise)
		self.X = [] # パーティクルセット set of particles
		self.loglikelihood = 0.0
		self.count = 0
		
		

	"""
	This method is called from Sensor class when new IMU sensor data are arrived.
	time_ : time (sec)
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
		dt = self.t - self.t1

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
			dt2 = 0.5 * dt * dt
			#dt3 = (1.0 / 6.0) * dt2 * dt
			A = np.array([[1.0,0.0,0.0,dt,0.0,0.0,dt2,0.0,0.0,0.0,0.0,0.0],
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
			Qt = np.diag([dt2,dt2,dt2,dt,dt,dt,1.0,1.0,1.0,dt,dt,dt])
			Q = Qt.dot(self.Q)
			self.mu, self.sigma = KF.execKF1Simple(Y,self.mu,self.sigma,A,self.C,Q,self.R)
			
			##################################
			# print variance of x
			#print("IMU"),
			#print(self.sigma[0][0]),
			#print(self.sigma[1][1]),
			#print(self.sigma[2][2])
			# print variance of a
			#print("IMU"),
			#print(self.sigma[6][6]),
			#print(self.sigma[7][7]),
			#print(self.sigma[8][8])
			##################################

		if(self.isFirstTimeIMU):
			self.isFirstTimeIMU = False



	"""
	This method is called from Image class when new camera image data are arrived.
	time_ : time (sec)
	keypointPairs : list of KeyPointPair class objects
	"""
	def setImageData(self, time_, keypointPairs):
				
		# If IMU data has not been arrived yet, do nothing
		if(self.isFirstTimeIMU):
			return
		
		# If first time, save mu and don't do anything else
		if(self.isFirstTimeCamera):
			self.isFirstTimeCamera = False
			self.mu1 = copy.deepcopy(self.mu) # save mu[t] as mu[t-1]
			return
		
		# Lock IMU process
		self.lock = True
		
		# Get current time
		self.t1 = self.t
		self.t = time_
		dt = self.t - self.t1
		
		# create particle from state vector
		self.X = self.createParticleFromStateVector(self.mu, self.sigma)
		
		# create X1 from mu[t-1]
		X1 = Particle()
		X1.initWithMu(self.mu1)
		
		###################
		print("====================")
		for X_ in self.X:
			print(X_.x[0]),
			print(X_.x[1]),
			print(X_.x[2])
		print("====================")
		###################
		
		# exec particle filter
		self.X = self.pf.pf_step(self.X, X1, dt, keypointPairs, self.M)
		
		###################
		print("------------------------")
		for X_ in self.X:
			print(X_.x[0]),
			print(X_.x[1]),
			print(X_.x[2])
		print("------------------------")
		###################
		
		# create state vector from particle set
		self.mu, self.sigma = self.createStateVectorFromParticle(self.X)
		
		##################################
		# print variance of x
		#print("Camera"),
		#print(self.sigma[0][0]),
		#print(self.sigma[1][1]),
		#print(self.sigma[2][2])
		# print variance of a
		#print("Camera"),
		#print(self.sigma[6][6]),
		#print(self.sigma[7][7]),
		#print(self.sigma[8][8])
		##################################
			
		# save mu[t] as mu[t-1]
		self.mu1 = copy.deepcopy(self.mu) 
		
		# Unlock IMU process
		self.lock = False


	"""
	create particle set from state vector
	"""
	def createParticleFromStateVector(self, mu, sigma):
		X = []
		for i in range(self.M):
			particle = Particle()
			particle.initWithStateVector(mu, sigma)
			X.append(particle)
		return X
		

	"""
	create state vector from particle set
	"""
	def createStateVectorFromParticle(self, X):
		x = []
		v = []
		a = []
		o = []
		for X_ in X:
			x.append(X_.x)
			v.append(X_.v)
			a.append(X_.a)
			o.append(X_.o)
		x_mu = np.mean(x, axis=0)
		v_mu = np.mean(v, axis=0)
		a_mu = np.mean(a, axis=0)
		o_mu = np.mean(o, axis=0)
		x_var = np.var(x, axis=0)
		v_var = np.var(v, axis=0)
		a_var = np.var(a, axis=0)
		o_var = np.var(o, axis=0)
		mu =  np.array([x_mu[0],x_mu[1],x_mu[2],v_mu[0],v_mu[1],v_mu[2],a_mu[0],a_mu[1],a_mu[2],o_mu[0],o_mu[1],o_mu[2]])
		sigma = np.diag([x_var[0],x_var[1],x_var[2],v_var[0],v_var[1],v_var[2],a_var[0],a_var[1],a_var[2],o_var[0],o_var[1],o_var[2]])
		return mu, sigma
		
	
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

