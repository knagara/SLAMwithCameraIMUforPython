# -*- coding: utf-8 -*-
"""
Particle Filter
(RBPF model version)
"""

import time
import numpy
import copy
import math
from particle import Particle
from landmark import Landmark
import KF


class ParticleFilterRBPF:

	def __init__(self):
		###########################
		self.count=0
		###########################
		pass

	def setFocus(self, f_):
		self.focus = f_

	def setParameter(self, param1, param2, param3):
		self.noise_a_sys = param1 # system noise of acceleration　加速度のシステムノイズ
		self.noise_g_sys = param2 # system noise of gyro　ジャイロのシステムノイズ
		self.noise_camera = param3 # observation noise of camera カメラの観測ノイズ
		
		self.R = numpy.array([[self.noise_camera, 0 ],
						[0, self.noise_camera]])

	def setObservationModel(self, observation_):
		self.observation = observation_

	def f_IMU(self, X, dt, dt2, accel, ori, noise):
		""" Transition model
		- 状態方程式
			x_t = f(x_t-1, u) + w
			w ~ N(0, sigma)
		"""
	
		X_new = Particle()
		X_new.landmarks = X.landmarks
	
		# Transition with noise (only x,v)		
		X_new.x = X.x + dt*X.v + dt2*X.a + dt2*noise
		#X_new.x = numpy.array([0.0,0.0,0.0]) ###################################
		X_new.v = X.v + dt*X.a + dt*noise
		X_new.a = accel
		X_new.o = ori
		#X_new.o = numpy.array([1.0,0.5,1.0]) ###################################
	
		return X_new


	def f_camera(self, X, dt, dt2, noise):
		""" Transition model
		- 状態方程式
			x_t = f(x_t-1, u) + w
			w ~ N(0, sigma)
		"""

		X_new = Particle()
		X_new.landmarks = X.landmarks
		
		# Transition with noise (only x,v)		
		X_new.x = X.x + dt*X.v + dt2*X.a + dt2*noise
		#X_new.x = numpy.array([0.0,0.0,0.0]) ###################################
		X_new.v = X.v + dt*X.a + dt*noise
		X_new.a = X.a
		X_new.o = X.o
		#X_new.o = numpy.array([1.0,0.5,1.0]) ###################################

		return X_new
	

	def likelihood(self, keypoints, step, P, X):
		""" Likelihood function
		- 尤度関数
			p(y|x) ~ exp(-1/2 * (|y-h(x)|.t * sigma * |y-h(x)|)
		- 観測モデル
			z = h(x) + v
			v ~ N(0, sigma)
		Parameters
		----------
		keypoints : 観測 Observation 特徴点 keypoints
		step : 現在のステップ数 current step
		x : 予測　Predicted particle
		Returns
		-------
		likelihood : 尤度 Likelihood
		"""
		
		likelihood = 0.0 # Likelihood
		
		xt = X.x[0]
		yt = X.x[1]
		zt = X.x[2]
		ox = X.o[0]
		oy = X.o[1]
		oz = X.o[2]
		
		for keypoint in keypoints:
			# previous landmark id
			prevLandmarkId = (step-1)*10000 + keypoint.prevIndex
			# new landmark id
			landmarkId = step*10000 + keypoint.index
			# The landmark is already observed or not?
			if(X.landmarks.has_key(prevLandmarkId) == False):
				# Fisrt observation
				# Initialize landmark and append to particle
				landmark = Landmark()
				landmark.init(X, keypoint, P, self.focus)
				X.landmarks[landmarkId] = landmark
			else:
				# Already observed
				X.landmarks[landmarkId] = X.landmarks[prevLandmarkId]
				del X.landmarks[prevLandmarkId]
				# Observation z				
				z = numpy.array([keypoint.x, keypoint.y])
				# Calc h and H (Jacobian matrix of h)
				xi = X.landmarks[landmarkId].mu[0]
				yi = X.landmarks[landmarkId].mu[1]
				zi = X.landmarks[landmarkId].mu[2]
				theta = X.landmarks[landmarkId].mu[3]
				phi = X.landmarks[landmarkId].mu[4]
				p = X.landmarks[landmarkId].mu[5]
				h = numpy.array([self.observation.fh1(self.focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p),
								self.observation.fh2(self.focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p)])
				H = numpy.array([[self.observation.fdh1xi(self.focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p), 
								self.observation.fdh1yi(self.focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p), 
								self.observation.fdh1zi(self.focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p), 
								self.observation.fdh1theta(self.focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p), 
								self.observation.fdh1phi(self.focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p), 
								self.observation.fdh1p(self.focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p)],
								[self.observation.fdh2xi(self.focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p), 
								self.observation.fdh2yi(self.focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p), 
								self.observation.fdh2zi(self.focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p), 
								self.observation.fdh2theta(self.focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p), 
								self.observation.fdh2phi(self.focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p), 
								self.observation.fdh2p(self.focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p)]])
				h_, H_ = X.landmarks[landmarkId].calcObservation(X, self.focus)
				
				###############################
				if(self.count == 0): ###############################
					print("z "),
					print(z)
					print("h "),
					print(h)
					print("h_ "),
					print(h_)
					print("H"),
					print(H)
					print("H_"),
					print(H_)
				###############################
					
				# Kalman filter (Landmark update)
				X.landmarks[landmarkId].mu, X.landmarks[landmarkId].sigma, S, Sinv = KF.execEKF1Update(z, h, X.landmarks[landmarkId].mu, X.landmarks[landmarkId].sigma, H, self.R)
				# Calc likelihood
				likelihood += (1.0 / (math.sqrt( numpy.linalg.det(2.0 * math.pi * S) ))) * numpy.exp( -0.5 * ( (z-h).T.dot(Sinv.dot(z-h)) ) )

		###############################
		#print("likelihood "+str(likelihood))
		#if(self.count == 0):
		#	print("likelihood "+str(likelihood))
		###########################
		self.count+=1
		###########################
		return likelihood


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
		for m in xrange(M):
			U = r + m * Minv
			while U > c:
				i += 1
				c += W[i]
			X_resampled.append(X[i])
			
		return X_resampled
		
	
	def predictionAndUpdate(self, X, dt, keypoints, step, P):
		
		# 推定 prediction
		dt2 = 0.5*dt*dt
		X_predicted = [self.f_camera(Xi, dt, dt2, numpy.random.normal(0, self.noise_a_sys, 3)) for Xi in X]
			
		###########################
		self.count=0
		###########################
		
		# 更新 update
		weight = [self.likelihood(keypoints, step, P, Xi) for Xi in X_predicted]
		
		return X_predicted, weight
			
		
	def normalizationAndResampling(self, X_predicted, weight, M):
		# 正規化 normalization of weight
		weight_sum = sum(weight) # 総和 the sum of weights
		if(weight_sum > 0.0005):
			# 重みの総和が大きい（尤度が高い）場合 likelihood is high enough
			print(weight_sum)     #########################
			# 正規化 normalization of weight
			weight = [(w/weight_sum) for w in weight]
			#for i in xrange(M):
			#	weight[i] /= weight_sum
			# リサンプリング re-sampling
			X_resampled = self.resampling(X_predicted, weight, M)
		else:
			# 重みの総和が小さい（尤度が低い）場合 likelihood is low
			print(weight_sum),    #########################
			
			print("***")          #########################
			# リサンプリングを行わない No re-sampling
			X_resampled = X_predicted
			
		return X_resampled


	def pf_step_camera(self, X, dt, keypoints, step, P, M):
		""" One Step of Sampling Importance Resampling for Particle Filter
			for IMU sensor
		Parameters
		----------
		X : 状態 List of state set
		dt : 時刻の差分 delta of time
		keypoints : 特徴点 keypoints
		step : 現在のステップ数 current step
		P : デバイス位置の分散共分散行列 Variance-covariance matrix of position
		M : パーティクルの数 num of particles
		Returns
		-------
		X_resampled : 次の状態 List updated state
		"""

		# 初期化 init
		X_predicted = range(M)
		weight = range(M)
		X_resampled = range(M)

		
		#############################
		start_time_ = time.clock() 
		#############################
		# 推定と更新 prediction and update
		X_predicted, weight = self.predictionAndUpdate(X, dt, keypoints, step, P)
		###############################
		end_time_ = time.clock()
		#print "update   time = %f" %(end_time_-start_time_) 
		###############################
		
		# 正規化とリサンプリング normalization and resampling
		X_resampled = self.normalizationAndResampling(X_predicted, weight, M)

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

		# 推定と更新 prediction and update
		# 観測がないので，パーティクルは予測だけで，更新されない
		# 予測をそのまま出力する
		dt2 = 0.5*dt*dt
		return [self.f_IMU(Xi, dt, dt2, accel, ori, numpy.random.normal(0, self.noise_a_sys, 3)) for Xi in X]
