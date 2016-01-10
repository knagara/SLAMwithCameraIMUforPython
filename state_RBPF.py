# -*- coding: utf-8 -*-

"""
state_RBPF.py

author: Keita Nagara　永良慶太 (University of Tokyo) <nagara.keita()gmail.com>

State and estimation model of Rao-Blackwellized Particle Filter.

This class is generated from "state.py".

"""

import sys
import math
import time
import copy
import datetime
import cv2 as cv
import numpy as np
from particle_filter import ParticleFilter
from particle import Particle
import Util

class StateRBPF:

	def __init__(self):
		# ----- Set parameters here! ----- #
		self.M = 100 # total number of particles パーティクルの数
		self.f = 924.1770935 # focus length of camera [px] カメラの焦点距離 [px]
		# Particle Filter
		self.noise_x_sys = 0.005 # system noise of position (SD)　位置のシステムノイズ（標準偏差）
		self.noise_x_sys_coefficient = 0.05 # system noise of position (coefficient)　位置のシステムノイズ（係数）
		self.noise_a_sys = 0.1 # system noise of acceleration (SD)　加速度のシステムノイズ（標準偏差）
		self.noise_g_sys = 0.01 # system noise of orientation (SD)　角度のシステムノイズ（標準偏差）
		self.noise_a_obs = 0.001 # observation noise of acceleration (SD)　加速度の観測ノイズ（標準偏差）
		self.noise_g_obs = 0.0001 # observation noise of orientation (SD)　角度の観測ノイズ（標準偏差）
		self.noise_camera = 10.0 # observation noise of camera (SD) カメラの観測ノイズ（標準偏差）
		self.noise_coplanarity = 0.1 # observation noise of coplanarity (SD) 共面条件の観測ノイズ（標準偏差）
		
		self.init()

	def init(self):
		self.isFirstTimeIMU = True
		self.isFirstTimeCamera = True
		self.lock = False

		self.t = 0.0
		self.t1 = 0.0
		self.t_camera = 0.0
		self.t1_camera = 0.0
		
		self.accel1 = np.array([0.0, 0.0, 0.0])
		self.accel2 = np.array([0.0, 0.0, 0.0])
		self.accel3 = np.array([0.0, 0.0, 0.0])
		
		self.P1 = np.identity(3)

		self.initParticleFilter()


	def initParticleFilter(self):
		self.pf = ParticleFilter().getParticleFilterClass("RBPF")
		self.pf.setFocus(self.f)
		self.pf.setParameter(self.noise_x_sys, self.noise_a_sys, self.noise_g_sys, self.noise_camera, self.noise_coplanarity, self.noise_x_sys_coefficient) #パーティクルフィルタのパラメータ（ノイズ） parameters (noise)
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
		
	
	def setObservationModel(self, observation_):
		self.pf.setObservationModel(observation_)



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
			# is Device Moving
			isMoving = [True, True, True]
			"""
			self.accel3 = copy.deepcopy(self.accel2)
			self.accel2 = copy.deepcopy(self.accel1)
			self.accel1 = copy.deepcopy(accel)
			if(Util.isDeviceMoving(self.accel1[0]) == False and Util.isDeviceMoving(self.accel2[0]) == False and Util.isDeviceMoving(self.accel3[0]) == False):
				isMoving[0] = False
			if(Util.isDeviceMoving(self.accel1[1]) == False and Util.isDeviceMoving(self.accel2[1]) == False and Util.isDeviceMoving(self.accel3[1]) == False):
				isMoving[1] = False
			if(Util.isDeviceMoving(self.accel1[2]) == False and Util.isDeviceMoving(self.accel2[2]) == False and Util.isDeviceMoving(self.accel3[2]) == False):
				isMoving[2] = False
			"""
			
			# exec particle filter
			self.X = self.pf.pf_step_IMU(self.X, self.dt, accel, ori, isMoving, self.M, self.isFirstTimeCamera)

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
		
		########################
		print("=================")
		print("step "+str(self.step)+"  count "+str(self.count))
		###########################
		
		if(keypoints == "nomatch"):
			print("nomatch      ***********************")
			self.reduce_particle_variance(self.X)
			self.count += 1
			self.step += 1
			return
		
		# Lock IMU process
		self.lock = True

		# Get current time
		self.t1 = self.t
		self.t = time_
		self.dt = self.t - self.t1
		
		self.t1_camera = self.t_camera
		self.t_camera = time_
		dt_camera = self.t_camera - self.t1_camera
		
		# covariance matrix of position
		P = self.createPositionCovarianceMatrixFromParticle(self.X)
		#P *= 0.01
		
		if(self.step > 0 and self.step < 10):
			#self.saveXYZasCSV(self.X,"1") 
			pass
		
		if(self.isFirstTimeCamera):
			# exec particle filter
			self.X = self.pf.pf_step_camera_firsttime(self.X, self.dt, keypoints, self.step, P, self.M)
		else:
			# exec particle filter
			self.X = self.pf.pf_step_camera(self.X, self.dt, keypoints, self.step, P, self.M, self.X1, self.P1, dt_camera)
		
		if(self.step > 0 and self.step < 10):
			#self.saveXYZasCSV(self.X,"2") 
			pass
		
		# Get prev position and orientation
		prevXx, prevXo = self.getPositionAndOrientation()
		self.X1 = Particle()
		self.X1.initWithPositionAndOrientation(prevXx, prevXo)
		
		self.P1 = P

		# Count
		self.count += 1
		
		# Step (camera only observation step)
		self.step += 1
		
		# Unlock IMU process
		self.lock = False
		
		if(self.isFirstTimeCamera):
			self.isFirstTimeCamera = False
		
		
	def reduce_particle_variance(self, X):
		"""
		This method is called when No-resampling = True.
		Reduce particle variance to avoid divergence of particles.
		"""
		
		x = []
		# Calc average of position
		for X_ in X:
			x.append(X_.x)
		average = np.mean(x, axis=0)
		
		# Reduce variance of position
		for X_ in X:
			difference = X_.x - average
			X_.x = average + difference * 0.1
			
		return X
		
		
	"""
	print Landmark (X,Y,Z)
	"""
	def printLandmark(self,X):
		print("-----")
		landmarks = self.getLandmarkXYZ(X)
		for key, value in landmarks.iteritems():
			print(str(key)+" "),
			print(value)
		
		
	"""
	return Landmark (X,Y,Z)
	"""
	def getLandmarkXYZ(self,X):
		allLandmarks = {}
		# calc sum of landmark XYZ
		for x in X:
			for landmarkId, landmark in x.landmarks.iteritems():
				xyz = landmark.getXYZ()
				if(allLandmarks.has_key(landmarkId) == False):
					allLandmarks[landmarkId] = xyz
				else:
					allLandmarks[landmarkId] += xyz
		# calc average of landamrk XYZ
		for key, value in allLandmarks.iteritems():
			value /= float(self.M)
		return allLandmarks
			
		
	"""
	print (X,Y,Z) of particles
	"""
	def printXYZ(self,X):
		print("-----")
		for x in X:
			x.printXYZ()
		
		
		
	"""
	save (X,Y,Z) of particles as CSV file
	"""
	def saveXYZasCSV(self,X,appendix):
		x = []
		for X_ in X:
			x.append(X_.x)
		date = datetime.datetime.now()
		#datestr = date.strftime("%Y%m%d_%H%M%S_") + "%04d" % (date.microsecond // 1000)
		#np.savetxt('./data/plot3d/'+datestr+'_xyz_'+appendix+'.csv', x, delimiter=',')
		datestr = date.strftime("%Y%m%d_%H%M%S_")
		np.savetxt('./data/output/particle_'+datestr+str(self.count)+'_'+appendix+'.csv', x, delimiter=',')


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
	return estimated state vector of position and orientation
	"""
	def getPositionAndOrientation(self):
		x = []
		o = []
		for X_ in self.X:
			x.append(X_.x)
			o.append(X_.o)
		return np.mean(x, axis=0),np.mean(o, axis=0)


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

