# -*- coding: utf-8 -*-
"""
Particle Filter
(IMUPF2 model version)

Difference between IMUPF and IMUPF2:
IMUPF -> IMU data is regarded as observation
IMUPF2 -> IMU data is regarded as control
"""

import numpy
import copy
import math
import Util

class ParticleFilterIMU2:

	def __init__(self):
		self.var_sys = 1.0
		self.var_obs = 1.0
		
	def setParameter(self, param1, param2):
		self.var_sys = param1
		self.var_obs = param2
	

	def f(self, dt, X, accel, ori):
		""" Transition model
		- 状態方程式
			x_t = f(x_t-1, u) + w
			w ~ N(0, sigma)
		"""
	
		w_mean = numpy.zeros(3) # mean of noise
		w_cov_a = numpy.eye(3) * self.var_sys # covariance matrix of noise (accel)
		w_a = numpy.random.multivariate_normal(w_mean, w_cov_a) # generate random
		w_cov_o = numpy.eye(3) * self.var_sys # covariance matrix of noise (ori)
		w_o = numpy.random.multivariate_normal(w_mean, w_cov_o) # generate random
		
		dt2 = dt*dt
		dt3 = dt*dt*dt
		
		X_new = copy.deepcopy(X)
			
		X_new.x = X.x + dt*X.v + 0.5*dt2*X.a + 0.5*dt2*w_a
		X_new.v = X.v + dt*X.a + dt*w_a
		X_new.a = accel
		X_new.o = ori
			
	
		return X_new
					

	def likelihood(self, y, x):
		""" Likelihood function
		- 尤度関数
			p(y|x) ~ exp(-1/2 * (|y-h(x)|.t * sigma * |y-h(x)|)
		- 観測モデル
			z = h(x) + v
			v ~ N(0, sigma)
		Parameters
		----------
		y : 観測 Observation
		x : 予測　Predicted particle
		Returns
		-------
		likelihood : 尤度 Likelihood
		"""
					
		return 1.0
					

	def resampling(self, X, W, M):
		""" Resampling
		- 等間隔サンプリング
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
					

	def pf_step(self, X, dt, accel, ori, M):
		""" One Step of Sampling Importance Resampling for Particle Filter
			for IMU sensor
		Parameters
		----------
		X : 状態 List of state set
		dt: 時刻の差分 delta of time
		accel : 制御 (加速度) Control (accel)
		ori : 制御 (姿勢) Control (orientation)
		M : パーティクルの数 num of particles
		Returns
		-------
		X_resampled : 次の状態 List updated state
		"""

		# 初期化
		X_predicted = range(M)
		weight = range(M)

		for i in range(M):
			# 推定 prediction
			X_predicted[i] = self.f(dt, X[i], accel, ori)
			# 更新 update
			#weight[i] = self.likelihood(y, X_predicted[i])
		# 正規化 normalization
		#weight_sum = sum(weight) # 総和 the sum of weights
		#for i in range(M):
		#    weight[i] /= weight_sum
		# リサンプリング re-sampling (if necessary)
		#X_resampled = self.resampling(X_predicted, weight, M)

		#観測がないので，パーティクルは予測だけで，更新されない
		X_resampled = X_predicted
		
		return X_resampled
