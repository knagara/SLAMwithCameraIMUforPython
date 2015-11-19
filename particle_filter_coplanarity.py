# -*- coding: utf-8 -*-
"""
Particle Filter
(Coplanarity model version)
"""

import numpy
import copy
import math
import Util

class ParticleFilterCoplanarity:

	def __init__(self):
		self.var_sys = 1.0
		self.var_obs = 1.0
		
	def setFocus(self, f_):
		self.focus = f_
		
	def setParameter(self, param1, param2, param3, param4, param5):
		self.noise_a_sys = param1 # system noise of acceleration　加速度のシステムノイズ
		self.noise_g_sys = param2 # system noise of gyro　ジャイロのシステムノイズ
		self.noise_a_obs = param3 # observation noise of acceleration　加速度の観測ノイズ
		self.noise_g_obs = param4 # observation noise of gyro　ジャイロの観測ノイズ
		self.noise_coplanarity_obs = param5 # observation noise of coplanarity 共面条件の観測ノイズ
	

	def f(self, dt, X):
		""" Transition model
		- 状態方程式
			x_t = f(x_t-1) + w
			w ~ N(0, sigma)
		"""

		X_new = copy.deepcopy(X)
		
		dt2 = 0.5 * dt * dt
		
		"""# Simple transition
		X_new.x = X.x + dt*X.v + dt2*X.a
		X_new.v = X.v + dt*X.a
		X_new.a = X.a
		X_new.o = X.o
		"""
		
		# Transition with noise (only x,v)
		w_mean = numpy.zeros(3) # mean of noise
		w_cov_a = numpy.eye(3) * self.noise_a_sys # covariance matrix of noise (accel)
		w_a = numpy.random.multivariate_normal(w_mean, w_cov_a) # generate random
		
		X_new.x = X.x + dt*X.v + dt2*X.a + dt2*w_a
		X_new.v = X.v + dt*X.a + dt*w_a
		X_new.a = X.a
		X_new.o = X.o
		
		
		"""  # Transition with noise
		w_mean = numpy.zeros(3) # mean of noise
		w_cov_a = numpy.eye(3) * self.noise_a_sys # covariance matrix of noise (accel)
		w_a = numpy.random.multivariate_normal(w_mean, w_cov_a) # generate random
		w_cov_g = numpy.eye(3) * self.noise_g_sys # covariance matrix of noise (gyro)
		w_g = numpy.random.multivariate_normal(w_mean, w_cov_g) # generate random
		
		X_new.x = X.x + dt*X.v + dt2*X.a + dt2*w_a
		X_new.v = X.v + dt*X.a + dt*w_a
		X_new.a = X.a + w_a
		X_new.o = X.o + dt*w_g
		"""

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
		
		# Generate coplanarity matrix and calc determinant.
		# | x-x1 y-y1 z-z1 | -> xyz
		# |  u1   v1   w1  | -> uvw1
		# |  u2   v2   w2  | -> uvw2
		coplanarity_determinant_square_total = 0.0
		xyz = numpy.array([X.x[0] - X1.x[0], X.x[1] - X1.x[1], X.x[2] - X1.x[2]])
		for KP in keypointPairs:
			# Generate uvw1 (time:t-1)
			uvf1 = numpy.array([KP.x1, KP.y1, self.focus])
			rotX = Util.rotationMatrixX(X1.o[0])
			rotY = Util.rotationMatrixY(X1.o[1])
			rotZ = Util.rotationMatrixZ(X1.o[2])
			# uvw1 = R(z)R(y)R(x)uvf1
			uvw1 = numpy.dot(rotZ,numpy.dot(rotY,numpy.dot(rotX,uvf1)))
			uvw1 /= 100.0 # Adjust scale to decrease calculation error. This doesn't have an influence to likelihood.
			# Generate uvw2 (time:t)
			uvf2 = numpy.array([KP.x2, KP.y2, self.focus])
			rotX = Util.rotationMatrixX(X.o[0])
			rotY = Util.rotationMatrixY(X.o[1])
			rotZ = Util.rotationMatrixZ(X.o[2])
			# uvw2 = R(z)R(y)R(x)uvf2
			uvw2 = numpy.dot(rotZ,numpy.dot(rotY,numpy.dot(rotX,uvf2)))
			uvw2 /= 100.0 # Adjust scale to decrease calculation error. This doesn't have an influence to likelihood.
			# Generate coplanarity matrix
			coplanarity_matrix = numpy.array([xyz,uvw1,uvw2])
			# Calc determinant
			determinant = numpy.linalg.det(coplanarity_matrix)
			# Add
			coplanarity_determinant_square_total += (determinant**2)
					
		#print(coplanarity_determinant_square_total)
					
		# return likelihood
		return numpy.exp(coplanarity_determinant_square_total / (-2.0 * (self.noise_coplanarity_obs**2)) )
					

	def resampling(self, X, W, M):
		""" Resampling
		- 等間隔サンプリング
		W : 重み weights
		M : パーティクルの数 num of particles
		"""
		X_resampled = []
		Minv = 1/float(M)
		r = numpy.random.rand() * Minv
		c = W[0]
		i = 0
		for m in range(M):
			U = r + m * Minv
			while U > c:
				i += 1
				c += W[i]
			X_resampled.append(X[i])
		return X_resampled
					

	def pf_step(self, X, X1, dt, keypointPairs, M):
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

		for i in range(M):
			# 推定 prediction
			X_predicted[i] = self.f(dt, X[i])
			# 更新 update
			weight[i] = self.likelihood(keypointPairs, X_predicted[i], X1)
		# 正規化 normalization of weight
		weight_sum = sum(weight) # 総和 the sum of weights
		if(weight_sum > 0.5):
			# 重みの総和が大きい（尤度が高い）場合 likelihood is high enough
			print(weight_sum)
			# 正規化 normalization of weight
			for i in range(M):
				weight[i] /= weight_sum
			# リサンプリング re-sampling
			X_resampled = self.resampling(X_predicted, weight, M)
		else:
			# 重みの総和が小さい（尤度が低い）場合 likelihood is low
			print(weight_sum),
			print("***")
			# リサンプリングを行わない No re-sampling
			X_resampled = copy.deepcopy(X_predicted)

		return X_resampled
