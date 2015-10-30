# -*- coding: utf-8 -*-
"""
Particle Filter
(Coplanarity model version)
"""

import numpy
import copy
import math

class ParticleFilterCoplanarity:

	def __init__(self):
		self.var_sys = 1.0
		self.var_obs = 1.0
		
	def setParameter(self, param1, param2, param3, param4):
		self.noise_a_sys = param1 # system noise of acceleration　加速度のシステムノイズ
		self.noise_g_sys = param2 # system noise of gyro　ジャイロのシステムノイズ
		self.noise_a_obs = param3 # observation noise of acceleration　加速度の観測ノイズ
		self.noise_g_obs = param4 # observation noise of gyro　ジャイロの観測ノイズ
	

	def f(self, dt, X):
		""" Transition model
		- 状態方程式
			x_t = f(x_t-1) + w
			w ~ N(0, sigma)
		"""

		w_mean = numpy.zeros(3) # mean of noise
		w_cov_a = numpy.eye(3) * self.noise_a_sys # covariance matrix of noise (accel)
		w_a = numpy.random.multivariate_normal(w_mean, w_cov_a) # generate random
		w_cov_g = numpy.eye(3) * self.noise_g_sys # covariance matrix of noise (gyro)
		w_g = numpy.random.multivariate_normal(w_mean, w_cov_g) # generate random

		dt2 = 0.5 * dt * dt
		dt3 = (1.0 / 6.0) * dt2 * dt

		X_new = copy.deepcopy(X)
		X_new.x = X.x + dt*X.v + dt2*X.a + dt3*w_a
		X_new.v = X.v + dt*X.a + dt2*w_a
		X_new.a = X.a + dt*w_a
		X_new.o = X.o + dt2*w_g

		return X_new
					

	def likelihood(self, keypoints, X, X1):
		""" Likelihood function
		- 尤度関数
			p(y|x) ~ exp(-|coplanarity|^2 / 2*sigma^2)
		Parameters
		----------
		keypoints : 特徴点ペア pairs of keyponts between t-1 frame and t frame
		X : 予測　Predicted particle
		X1 : t-1の状態　particle at time t-1
		Returns
		-------
		likelihood : 尤度 Likelihood
		"""
		
		xyz = numpy.array([X.x[0] - X1.x[0], X.x[1] - X1.x[1], X.x[2] - X1.x[2]])
		uvw1 = numpy.array([0.,0.,0.])
		uvw2 = numpy.array([0.,0.,0.])
		
		coplanarity = numpy.array([xyz,uvw1,uvw2])
		det = numpy.linalg.det(coplanarity)
					
		return numpy.exp()
					

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
					

	def pf_step(self, X, dt, keypoints, M):
		""" One Step of Sampling Importance Resampling for Particle Filter
			for IMU sensor
		Parameters
		----------
		X : 状態 List of state set
		dt : 時刻の差分 delta of time
		keypoints : 特徴点ペア pairs of keyponts between t-1 frame and t frame
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
			weight[i] = self.likelihood(keypoints, X_predicted[i], X[i])
		# 正規化 normalization
		weight_sum = sum(weight) # 総和 the sum of weights
		for i in range(M):
			weight[i] /= weight_sum
		# リサンプリング re-sampling
		X_resampled = self.resampling(X_predicted, weight, M)

		return X_resampled
