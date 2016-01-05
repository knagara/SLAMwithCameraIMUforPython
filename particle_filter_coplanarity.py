# -*- coding: utf-8 -*-
"""
Particle Filter
(Coplanarity model version)
"""

import numpy as np
import copy
import math
import datetime
import Util

class ParticleFilterCoplanarity:

	def __init__(self):
		pass
		
	def setFocus(self, f_):
		self.focus = f_
		
	def setParameter(self, param1, param2):
		self.noise_x_sys = param1 # system noise of position　位置のシステムノイズ
		self.noise_coplanarity = param2 # observation noise of coplanarity 共面条件の観測ノイズ

	def f(self, dt, dt2, dt_camera, X, X1):
		""" Transition model
		- 状態方程式
			x_t = f(x_t-1) + w
			w ~ N(0, sigma)
		"""

		X_new = copy.deepcopy(X)
		
		# Transition with noise
		#w_mean = np.zeros(3) # mean of noise
		#w_cov_x = np.eye(3) * self.noise_x_sys # covariance matrix of noise (accel)
		#w_x = np.random.multivariate_normal(w_mean, w_cov_x) # generate random
		w_x = np.random.normal(0, self.noise_x_sys, 3)
		
		X_new.x = X.x + dt*X.v + dt2*X.a + w_x
		X_new.v = (X_new.x - X1.x)/dt_camera
		X_new.a = X.a
		X_new.o = X.o

		return X_new
					

	def likelihood(self, keypointPairs, X, X1):
		""" Likelihood function
		- 尤度関数
			p(y|x) ~ exp(-|coplanarity|^2 / 2*sigma^2)
		Parameters
		----------
		keypointPairs : 特徴点ペア pairs of keyponts between t-1 frame and t frame
		X : 予測　Predicted particle
		X1 : t-1の状態　particle at time t-1
		Returns
		-------
		likelihood : 尤度 Likelihood
		"""
		
		weight = 0.0 # weight (return value)
		weights = []
		
		#total_keypoints = len(keypointPairs)
		
		# Generate coplanarity matrix and calc determinant.
		# | x-x1 y-y1 z-z1 | -> xyz
		# |  u1   v1   w1  | -> uvw1
		# |  u2   v2   w2  | -> uvw2
		
		#coplanarity_determinant_square_total = 0.0
		xyz = np.array([X.x[0] - X1.x[0], X.x[1] - X1.x[1], X.x[2] - X1.x[2]])
		for KP in keypointPairs:
			# Generate uvw1 (time:t-1)
			uvf1 = np.array([KP.x1, -KP.y1, -self.focus]) # Camera coordinates -> Device coordinates
			rotX = Util.rotationMatrixX(X1.o[0])
			rotY = Util.rotationMatrixY(X1.o[1])
			rotZ = Util.rotationMatrixZ(X1.o[2])
			# uvw1 = R(z)R(y)R(x)uvf1
			uvw1 = np.dot(rotZ,np.dot(rotY,np.dot(rotX,uvf1)))
			uvw1 /= 100.0 # Adjust scale to decrease calculation error. This doesn't have an influence to estimation.
			# Generate uvw2 (time:t)
			uvf2 = np.array([KP.x2, -KP.y2, -self.focus]) # Camera coordinates -> Device coordinates
			rotX = Util.rotationMatrixX(X.o[0])
			rotY = Util.rotationMatrixY(X.o[1])
			rotZ = Util.rotationMatrixZ(X.o[2])
			# uvw2 = R(z)R(y)R(x)uvf2
			uvw2 = np.dot(rotZ,np.dot(rotY,np.dot(rotX,uvf2)))
			uvw2 /= 100.0 # Adjust scale to decrease calculation error. This doesn't have an influence to estimation.
			# Generate coplanarity matrix
			coplanarity_matrix = np.array([xyz,uvw1,uvw2])
			# Calc determinant
			determinant = np.linalg.det(coplanarity_matrix)
			# Weight
			w = (1.0 / (math.sqrt( 2.0 * math.pi * self.noise_coplanarity**2 ))) * np.exp((determinant**2) / (-2.0 * (self.noise_coplanarity**2)) )
			weights.append(w)
			# Add
			#coplanarity_determinant_square_total += (determinant**2)
					
		
		for i,w in enumerate(weights):
			if(i==0):
				weight = w
			else:
				weight *= w
		
		
		#weight = (1.0 / (math.sqrt( 2.0 * math.pi * (self.noise_coplanarity*total_keypoints)**2 ))) * np.exp(coplanarity_determinant_square_total / (-2.0 * ((self.noise_coplanarity*total_keypoints)**2)) )
				
		###########################################
		#print("weight"),
		#print(weight)
		###########################################
		
		# return likelihood
		return weight 
					

	def resampling(self, X, W, M):
		""" Resampling
		- 等間隔サンプリング
		W : 重み weights
		M : パーティクルの数 num of particles
		"""
		X_resampled = []
		Minv = 1/float(M)
		r = np.random.rand() * Minv
		c = W[0]
		i = 0
		for m in range(M):
			U = r + m * Minv
			while U > c:
				i += 1
				c += W[i]
			X_resampled.append(X[i])
		return X_resampled
					

	def pf_step(self, X, X1, dt, dt_camera, keypointPairs, M):
		""" One Step of Sampling Importance Resampling for Particle Filter
			for IMU sensor
		Parameters
		----------
		X : 状態 List of state set
		X1 : 前回の状態 state at time t-1
		dt : 時刻の差分 delta of time
		keypointPairs : 特徴点ペア pairs of keypoints between t-1 frame and t frame
		M : パーティクルの数 num of particles
		Returns
		-------
		X_resampled : 次の状態 List updated state
		"""

		# 初期化 init
		X_predicted = range(M)
		weight = range(M)
		
		dt2 = 0.5 * dt * dt

		for i in range(M):
			# 推定 prediction
			X_predicted[i] = self.f(dt, dt2, dt_camera, X[i], X1)
			# 更新 update
			weight[i] = self.likelihood(keypointPairs, X_predicted[i], X1)		
			
		# 正規化 normalization of weight
		weight_sum = sum(weight) # 総和 the sum of weights
		if(weight_sum > 0.0):
			# 重みの総和が大きい（尤度が高い）場合 likelihood is high enough
			print("weight_sum "+str(weight_sum))
			# 正規化 normalization of weight
			for i in range(M):
				weight[i] /= weight_sum
			# リサンプリング re-sampling
			X_resampled = self.resampling(X_predicted, weight, M)
		else:
			# 重みの総和が小さい（尤度が低い）場合 likelihood is low
			print("weight_sum "+str(weight_sum)),
			print("***")
			# リサンプリングを行わない No re-sampling
			X_resampled = copy.deepcopy(X_predicted)

		return X_resampled
