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
		self.noise_a_sys_camera = param3 # system noise of acceleration (at camera step) 加速度のシステムノイズ(カメラ観測時)
		self.noise_camera = param4 # observation noise of camera カメラの観測ノイズ
		
		self.dt_camera = 0.1 # time delta at camera step カメラ観測時の Δt
		self.dt2_camera = 0.5 * self.dt_camera * self.dt_camera # time delta at camera step カメラ観測時の 1/2*Δt^2
		
		self.Q = 0.5 * self.dt_camera * self.dt_camera * np.diag([self.noise_a_sys_camera, self.noise_a_sys_camera, self.noise_a_sys_camera])
						
		self.R = np.diag([self.noise_camera, self.noise_camera])

	def setObservationModel(self, observation_):
		self.observation = observation_


	def f_IMU(self, X, dt, dt2, accel, ori, isMoving, noise):
		""" Transition model
		- 状態方程式
			x_t = f(x_t-1, u) + w
			w ~ N(0, sigma)
		"""
		
		if(isMoving[0] == False):
			X.v[0] = 0.0
		if(isMoving[1] == False):
			X.v[1] = 0.0
		if(isMoving[2] == False):
			X.v[2] = 0.0
	
		X_new = Particle()
		X_new.landmarks = X.landmarks
	
		# Transition with noise (only x,v)		
		X_new.x = X.x + dt*X.v + dt2*X.a + dt2*noise
		X_new.v = X.v + dt*X.a + dt*noise
		X_new.a = accel
		X_new.o = ori
		
		#if(isMoving[0] == False):
		#	X_new.v[0] = 0.0
		#if(isMoving[1] == False):
		#	X_new.v[1] = 0.0
		#if(isMoving[2] == False):
		#	X_new.v[2] = 0.0
	
		return X_new
	

	def predictionAndUpdateOneParticle(self, X, dt, dt2, noise, keypoints, step, P):
		
		"""
		FastSLAM1.0
		"""
		
		weight = 0.0 # weight (return value)
		weights = []
		count_of_known_keypoints = 0
		
		# 姿勢予測 prediction of position
		X_ = Particle()
		X_.landmarks = X.landmarks
		X_.x = X.x + dt*X.v + dt2*X.a + self.dt2_camera*noise
		X_.v = X.v + dt*X.a + self.dt_camera*noise
		X_.a = X.a
		X_.o = X.o
		
		for keypoint in keypoints:
			# previous landmark id
			prevLandmarkId = (step-1)*10000 + keypoint.prevIndex
			# new landmark id
			landmarkId = step*10000 + keypoint.index
			# The landmark is already observed or not?
			if(X_.landmarks.has_key(prevLandmarkId) == False):
				# Fisrt observation
				# Initialize landmark and append to particle
				landmark = Landmark()
				landmark.init(X_, keypoint, P, self.focus)
				X_.landmarks[landmarkId] = landmark
			else:
				# Already observed
				count_of_known_keypoints += 1
				X_.landmarks[landmarkId] = X_.landmarks[prevLandmarkId]
				del X_.landmarks[prevLandmarkId]
				# Observation z				
				z = np.array([keypoint.x, keypoint.y])
				# Calc h and H (Jacobian matrix of h)
				h, Hx, H = X_.landmarks[landmarkId].calcObservation(X_, self.focus)
				# Kalman filter (Landmark update)
				X_.landmarks[landmarkId].mu, X_.landmarks[landmarkId].sigma, S, Sinv = KF.execEKF1Update(z, h, X_.landmarks[landmarkId].mu, X_.landmarks[landmarkId].sigma, H, self.R)
				# Calc weight
				w = 0.0
				try:
					w = (1.0 / (math.sqrt( np.linalg.det(2.0 * math.pi * S) ))) * np.exp( -0.5 * ( (z-h).T.dot(Sinv.dot(z-h)) ) )
				except:
					print("Error on calc weight: ")
				weights.append(w)
				###############################
				if(self.count == 0):
					#print("z "),
					#print(z)
					#print("h "),
					#print(h)
					#print((math.sqrt( np.linalg.det(2.0 * math.pi * S) )))
					pass
				###############################
					
		for w in weights:
			weight += w

		"""
		if(count_of_known_keypoints > 0):
			for i in xrange(len(weights)):
				if(i==0):
					weight = (100*w)
				else:
					weight *= (100*w)
		"""
			
		###############################
		#print("weight "+str(weight))
		if(self.count == 0):
			#print("weight "+str(weight))
			pass
		###########################
		self.count+=1
		###########################
		return X_, weight
		
	
	def predictionAndUpdateOneParticle2(self, X, dt, dt2, keypoints, step, P):
		
		"""
		FastSLAM2.0
		"""
		
		RSS = [] # residual sum of squares
		weight = 0.0 # weight (return value)
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
			if(X_.landmarks.has_key(prevLandmarkId) == False):
				# Fisrt observation
				# Initialize landmark and append to particle
				landmark = Landmark()
				landmark.init(X, keypoint, P, self.focus)
				X_.landmarks[landmarkId] = landmark
			else:
				# Already observed
				count_of_known_keypoints += 1
				X_.landmarks[landmarkId] = X_.landmarks[prevLandmarkId]
				del X_.landmarks[prevLandmarkId]
				
				# Actual observation z				
				z = np.array([keypoint.x, keypoint.y])
				
				# 計測予測 prediction of observation
				#  Calc h(x), Hx, Hm (Jacobian matrix of h with respect to X and Landmark)
				z__, Hx, Hm = X_.landmarks[landmarkId].calcObservation(X_, self.focus)
				
				# 姿勢のカルマンフィルタ Kalman filter of position
				"""
				mu_ = copy.deepcopy(X_.landmarks[landmarkId].mu)
				sigma_ = copy.deepcopy(X_.landmarks[landmarkId].sigma)
				
				S = Hm.dot(sigma_.dot(Hm.T)) + self.R
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
				z_ = X_.landmarks[landmarkId].h(X_.x + x_diff, X_.o, self.focus)
				
				# ランドマークの予測 prediction of landmark
				K = X_.landmarks[landmarkId].sigma.dot(Hm.T.dot(Sinv))
				X_.landmarks[landmarkId].mu += K.dot(z - z_)
				X_.landmarks[landmarkId].sigma = X_.landmarks[landmarkId].sigma - K.dot(Hm.dot(X_.landmarks[landmarkId].sigma))
				"""
				
				S = Hm.dot(X_.landmarks[landmarkId].sigma.dot(Hm.T)) + self.R
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
				z_, XYZ = X_.landmarks[landmarkId].h(X_.x + x_diff, X_.o, self.focus)
				
				# ランドマークの予測 prediction of landmark
				K = X_.landmarks[landmarkId].sigma.dot(Hm.T.dot(Sinv))
				X_.landmarks[landmarkId].mu += K.dot(z - z_)
				X_.landmarks[landmarkId].sigma = X_.landmarks[landmarkId].sigma - K.dot(Hm.dot(X_.landmarks[landmarkId].sigma))
				
				
				# 重み計算 calc weight
				rss_ = (z-z_).T.dot(Linv.dot(z-z_))
				RSS.append(rss_)
				w = (1.0 / (math.sqrt( np.linalg.det(2.0 * math.pi * L) ))) * np.exp( -0.5 * ( rss_ ) )
				weights.append(w)
				
				###############################
				end_time_ = time.clock()
				if(self.count == 0):
					#print(XYZ)
					#print(x_diff)
					#print ""+str(landmarkId)+" update time = %f" %(end_time_-start_time_)
					pass
				###############################
					

		if(count_of_known_keypoints > 0):
			# 残差が最小のときの重みと位置を採用する
			max_index = RSS.index(min(RSS))
			weight = weights[max_index]
			weight *= 1000
			X_.x += x_diffs[max_index]
			X_.v += (2.0*x_diffs[max_index])/self.dt_camera
		
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
			X_predicted[i], weight[i] = self.predictionAndUpdateOneParticle(X[i], dt, dt2, np.random.normal(0, self.noise_a_sys_camera, 3), keypoints, step, P)
			#X_predicted[i], weight[i] = self.predictionAndUpdateOneParticle2(X[i], dt, dt2, keypoints, step, P)
		
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
			X_resampled.append(copy.deepcopy(X[i]))
			
		return X_resampled
			
		
	def normalizationAndResampling(self, X_predicted, weight, M):
		# 正規化 normalization of weight
		weight_sum = sum(weight) # 総和 the sum of weights
		if(weight_sum > 0.0):
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
		#print "update   time = %f" %(end_time_-start_time_) 
		###############################
		
		# 正規化とリサンプリング normalization and resampling
		X_resampled = self.normalizationAndResampling(X_predicted, weight, M)

		return X_resampled


	def pf_step_IMU(self, X, dt, accel, ori, isMoving, M):
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
		return [self.f_IMU(Xi, dt, dt2, accel, ori, isMoving, np.random.normal(0, self.noise_a_sys, 3)) for Xi in X]
