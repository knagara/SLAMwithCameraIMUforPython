# -*- coding: utf-8 -*-
"""
Particle Filter
(RBPF model version)
"""

import numpy
import copy
import math

class ParticleFilterRBPF:

	def __init__(self):
		self.var_sys = 1.0
		self.var_obs = 1.0

	def setFocus(self, f_):
		self.focus = f_

	def setParameter(self, param1, param2):
		self.noise_a_sys = param1 # system noise of acceleration　加速度のシステムノイズ
		self.noise_g_sys = param2 # system noise of gyro　ジャイロのシステムノイズ


	def f(self, dt, X, accel, ori):
		""" Transition model
		- 状態方程式
			x_t = f(x_t-1, u) + w
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


	def pf_step_camera(self, X, dt, keypointPairs, M):
		""" One Step of Sampling Importance Resampling for Particle Filter
			for IMU sensor
		Parameters
		----------
		X : 状態 List of state set
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
			weight[i] = self.likelihood(keypointPairs, X_predicted[i])
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


	def pf_step_IMU(self, X, dt, accel, ori, M):
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
