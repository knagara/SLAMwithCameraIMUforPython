# -*- coding: utf-8 -*-
"""
Particle Filter
(RBPF model version)
"""

import time
import numpy as np
import copy
import math
from particle import Particle
from landmark import Landmark
import KF


class ParticleFilterRBPF:

	def __init__(self):
		pass

	def setFocus(self, f_):
		self.focus = f_

	def setParameter(self, param1, param2, param3, param4):
		self.noise_a_sys = param1 # system noise of acceleration　加速度のシステムノイズ
		self.noise_g_sys = param2 # system noise of gyro　ジャイロのシステムノイズ
		self.noise_p_sys_camera = param3 # system noise of position　位置のシステムノイズ(カメラ観測時)
		self.noise_camera = param4 # observation noise of camera カメラの観測ノイズ
		
		self.Q = np.diag([self.noise_p_sys_camera, self.noise_p_sys_camera, self.noise_p_sys_camera])
						
		self.R = np.diag([self.noise_camera, self.noise_camera])

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
		#X_new.x = np.array([0.0,0.0,0.0]) ###################################
		X_new.v = X.v + dt*X.a + dt*noise
		X_new.a = accel
		X_new.o = ori
		#X_new.o = np.array([1.0,0.5,1.0]) ###################################
	
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
		#X_new.x = np.array([0.0,0.0,0.0]) ###################################
		X_new.v = X.v + dt*X.a + dt*noise
		X_new.a = X.a
		X_new.o = X.o
		#X_new.o = np.array([1.0,0.5,1.0]) ###################################

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
		
		for keypoint in keypoints:
			# previous landmark id
			prevLandmarkId = (step-1)*10000 + keypoint.prevIndex
			# new landmark id
			landmarkId = step*10000 + keypoint.index
			# The landmark is already observed or not?
			#############################
			start_time_ = time.clock() 
			#############################
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
				z = np.array([keypoint.x, keypoint.y])
				# Calc h and H (Jacobian matrix of h)
				h, H = X.landmarks[landmarkId].calcObservation(X, self.focus)
				# Kalman filter (Landmark update)
				X.landmarks[landmarkId].mu, X.landmarks[landmarkId].sigma, S, Sinv = KF.execEKF1Update(z, h, X.landmarks[landmarkId].mu, X.landmarks[landmarkId].sigma, H, self.R)
				# Calc likelihood
				likelihood += (1.0 / (math.sqrt( np.linalg.det(2.0 * math.pi * S) ))) * np.exp( -0.5 * ( (z-h).T.dot(Sinv.dot(z-h)) ) )
				###############################
				end_time_ = time.clock() #####################
				if(self.count == 0): ###############################
					pass
					#print ""+str(landmarkId)+" update time = %f" %(end_time_-start_time_) #####################
				###############################

		###############################
		#print("likelihood "+str(likelihood))
		#if(self.count == 0):
		#	print("likelihood "+str(likelihood))
		###########################
		self.count+=1
		###########################
		return likelihood
		
	
	def predictionAndUpdateOneParticle(self, X, dt, dt2, keypoints, step, P):
		
		weight = 0.0	
		weights = []
		count_of_known_keypoints = 0
		x_diff_sum = np.array([0.0, 0.0, 0.0])
		x_diffs = []
		
		# 姿勢予測 prediction of position
		X_ = Particle()
		X_.landmarks = X.landmarks
		X_.x = X.x + dt*X.v + dt2*X.a
		X_.v = X.v + dt*X.a
		X_.a = X.a
		X_.o = X.o
		
		for keypoint in keypoints:
			#############################
			start_time_ = time.clock() 
			#############################
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
				count_of_known_keypoints += 1
				X.landmarks[landmarkId] = X.landmarks[prevLandmarkId]
				del X.landmarks[prevLandmarkId]
				
				# Actual observation z				
				z = np.array([keypoint.x, keypoint.y])
				
				# 計測予測 prediction of observation
				#  Calc h(x), Hx, Hm (Jacobian matrix of h with respect to X and Landmark)
				z__, Hx, Hm = X.landmarks[landmarkId].calcObservation(X_, self.focus)
				
				# 姿勢のカルマンフィルタ Kalman filter of position
				S = Hm.dot(X.landmarks[landmarkId].sigma.dot(Hm.T)) + self.R
				L = S + Hx.dot(self.Q.dot(Hx.T))
				Sinv = np.linalg.inv(S)
				Linv = np.linalg.inv(L)
				sigmax = np.linalg.inv( Hx.T.dot(Sinv.dot(Hx)) + np.linalg.inv(self.Q) )
				mux = sigmax.dot(Hx.T.dot(Sinv.dot(z - z__)))
				
				# 姿勢のサンプリング sampling of position
				x_diff = np.random.multivariate_normal(mux, sigmax)
				x_diffs.append(x_diff)
				x_diff_sum += x_diff
				
				# 計測再予測 reprediction of observation
				z_ = X.landmarks[landmarkId].h(X_.x + x_diff, X.o, self.focus)
				
				# ランドマークの予測 prediction of landmark
				K = X.landmarks[landmarkId].sigma.dot(Hm.T.dot(Sinv))
				X.landmarks[landmarkId].mu += K.dot(z - z_)
				X.landmarks[landmarkId].sigma = X.landmarks[landmarkId].sigma - K.dot(Hm.dot(X.landmarks[landmarkId].sigma))
				
				# 重み計算 calc weight
				w = (1.0 / (math.sqrt( np.linalg.det(2.0 * math.pi * L) ))) * np.exp( -0.5 * ( (z-z_).T.dot(Linv.dot(z-z_)) ) )
				weights.append(w)
				
				###############################
				end_time_ = time.clock()
				if(self.count == 0):
					#print(x_diff)
					#print ""+str(landmarkId)+" update time = %f" %(end_time_-start_time_)
					pass
				###############################
					

		if(count_of_known_keypoints > 0):
			# 重みが最大のものだけを使用する
			max_index = weights.index(max(weights))
			weight = weights[max_index]
			weight *= 1000
			X_.x += x_diffs[max_index]
			X_.v += (2.0*x_diffs[max_index])/dt
		
		###############################
		#print("weight "+str(weight))
		#if(self.count == 0):
			#print("weight "+str(weight))
		###########################
		self.count+=1
		###########################
		
		return X_, weight
		
		
	
	def predictionAndUpdate(self, X, dt, keypoints, step, P, M):
			
		###########################
		self.count=0
		###########################
		
		X_predicted = range(M)
		weight = range(M)
		
		dt2 = 0.5*dt*dt
		
		for i in xrange(M):
			X_predicted[i], weight[i] = self.predictionAndUpdateOneParticle(X[i], dt, dt2, keypoints, step, P)
		
		return X_predicted, weight


	def resampling(self, X, W, M):
		""" Resampling
		- 等間隔サンプリング
		M : パーティクルの数 num of particles
		"""
		X_resampled = []
		Minv = 1.0/float(M)
		r = np.random.rand() * Minv
		c = W[0]
		i = 0
		for m in xrange(M):
			U = r + m * Minv
			while U > c:
				i += 1
				c += W[i]
			X_resampled.append(X[i])
			
		return X_resampled
			
		
	def normalizationAndResampling(self, X_predicted, weight, M):
		# 正規化 normalization of weight
		weight_sum = sum(weight) # 総和 the sum of weights
		if(weight_sum > 0.000001):
			# 重みの総和が大きい（尤度が高い）場合 likelihood is high enough
			print("weight_sum "+str(weight_sum))     #########################
			# 正規化 normalization of weight
			weight = [(w/weight_sum) for w in weight]
			#for i in xrange(M):
			#	weight[i] /= weight_sum
			# リサンプリング re-sampling
			X_resampled = self.resampling(X_predicted, weight, M)
		else:
			# 重みの総和が小さい（尤度が低い）場合 likelihood is low
			print("weight_sum "+str(weight_sum)),    #########################
			
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
		#X_predicted = range(M)
		#weight = range(M)
		X_resampled = range(M)

		
		#############################
		start_time_ = time.clock() 
		#############################
		# 推定と更新 prediction and update
		X_predicted, weight = self.predictionAndUpdate(X, dt, keypoints, step, P, M)
		###############################
		end_time_ = time.clock()
		print "update   time = %f" %(end_time_-start_time_) 
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
		return [self.f_IMU(Xi, dt, dt2, accel, ori, np.random.normal(0, self.noise_a_sys, 3)) for Xi in X]
