# -*- coding: utf-8 -*-
"""
Particle Filter
(RBPF model version)
"""

import time
import datetime
import numpy as np
import copy
import math
from particle import Particle
from landmark import Landmark
import KF
import Util


class ParticleFilterRBPF:

	def __init__(self):
		pass

	def setFocus(self, f_):
		self.focus = f_

	def setParameter(self, param1, param2, param3, param4, param5, param6):
		self.noise_x_sys = param1 # system noise of position (SD)　位置のシステムノイズ（標準偏差）
		self.noise_a_sys = param2 # system noise of acceleration (SD)　加速度のシステムノイズ（標準偏差）
		self.noise_g_sys = param3 # system noise of orientation (SD)　角度のシステムノイズ（標準偏差）
		self.noise_camera = param4 # observation noise of camera (SD) カメラの観測ノイズ（標準偏差）
		self.noise_coplanarity = param5 # observation noise of coplanarity (SD) 共面条件の観測ノイズ（標準偏差）
		self.noise_x_sys_coefficient = param6 #パーティクルフィルタのパラメータ（ノイズ） parameters (noise)
				
		self.Q = np.diag([self.noise_x_sys**2, self.noise_x_sys**2, self.noise_x_sys**2])
						
		self.R = np.diag([self.noise_camera**2, self.noise_camera**2])
		self.Rinv = np.diag([1.0/(self.noise_camera**2), 1.0/(self.noise_camera**2)])

	def setObservationModel(self, observation_):
		self.observation = observation_


	def f_IMU(self, X, dt, dt2, accel, ori, isMoving, noise):
		""" Transition model
		- 状態方程式
			x_t = f(x_t-1, u) + w
			w ~ N(0, sigma)
		"""
	
		X_new = Particle()
		X_new.landmarks = X.landmarks
	
		# Transition with noise (only x,v)		
		X_new.x = X.x + dt*X.v + dt2*X.a + noise
		X_new.v = X.v + dt*X.a
		#X_new.v = X.v + dt*X.a + noise*25
		#X_new.v = X.v + dt*X.a + noise*50
		X_new.a = accel
		X_new.o = ori
	
		return X_new
	

	def predictionAndUpdateOneParticle_firsttime(self, X, dt, dt2, keypoints, step, P):
		
		"""
		FastSLAM1.0
		"""
		
		weight = 0.0 # weight (return value)
		weights = []
		
		# 姿勢予測 prediction of position
		X_ = Particle()
		X_.landmarks = X.landmarks
		X_.x = X.x + dt*X.v + dt2*X.a
		X_.v = X.v + dt*X.a
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
				print("Error on pf_step_camera_firsttime")
				
		self.count+=1
		
		return X_, weight
	

	def predictionAndUpdateOneParticle(self, X, dt, dt2, noise, keypoints, step, P, X1, P1, dt_camera, noise_o):
		
		"""
		FastSLAM1.0
		"""
		
		weight = 0.0 # weight (return value)
		weights = []
		count_of_first_observation = 0
		
		# 姿勢予測 prediction of position
		X_ = Particle()
		X_.landmarks = X.landmarks
		X_.x = X.x + dt*X.v + dt2*X.a + noise
		#X_.v = X.v + dt*X.a
		#X_.v = X.v + dt*X.a + noise*25
		#X_.v = X.v + dt*X.a + noise*50
		X_.a = X.a
		X_.o = X.o
		#X_.o = X.o + noise_o
		
		# 速度調整 velocity adjustment
		#X_.v = ((X_.x - X1.x)/dt_camera)
		X_.v = ((X_.x - X1.x)/dt_camera)*0.25
		#X_.v = ((X_.x - X1.x)/dt_camera)*0.5
		
		# 共面条件モデルのための計算 Calc for Coplanarity
		xyz = np.array([X_.x[0] - X1.x[0], X_.x[1] - X1.x[1], X_.x[2] - X1.x[2]])
		
		# 前回ランドマークの全てのキーを取得しておく　あとで消す
		prevLandmarkKeys = []
		for key in X_.landmarks:
			prevLandmarkKeys.append(key)
		
		for keypoint in keypoints:
			
			# ---------------------------- #
			# Calc weight of Inverse Depth #
			# ---------------------------- #
			# previous landmark id
			prevLandmarkId = (step-1)*10000 + keypoint.prevIndex
			# new landmark id
			landmarkId = step*10000 + keypoint.index
			# The landmark is already observed or not?
			if(X_.landmarks.has_key(prevLandmarkId) == False):
				# Fisrt observation
				# Initialize landmark and append to particle
				landmark = Landmark()
				landmark.initPrev(X1, keypoint, P1, self.focus)
				X_.landmarks[landmarkId] = landmark
				if(self.count == 0):
					count_of_first_observation += 1
			else:
				# Already observed
				X_.landmarks[landmarkId] = X_.landmarks[prevLandmarkId]
				#del X_.landmarks[prevLandmarkId]
			# Update landmark and calc weight
			# Observation z				
			z = np.array([keypoint.x, keypoint.y])
			# Calc h and H (Jacobian matrix of h)
			h, Hx, H = X_.landmarks[landmarkId].calcObservation(X_, self.focus)
			# Kalman filter (Landmark update)
			S = H.dot(X_.landmarks[landmarkId].sigma.dot(H.T)) + self.R
			Sinv = np.linalg.inv(S)
			K = X_.landmarks[landmarkId].sigma.dot(H.T.dot(Sinv))
			X_.landmarks[landmarkId].mu += K.dot(z - h)
			X_.landmarks[landmarkId].sigma = X_.landmarks[landmarkId].sigma - K.dot(H.dot(X_.landmarks[landmarkId].sigma))
			# Calc weight
			w = 0.0
			try:
				#w = (1.0 / (math.sqrt( np.linalg.det(2.0 * math.pi * S) ))) * np.exp( -0.5 * ( (z-h).T.dot(Sinv.dot(z-h)) ) )
				#w = (1.0 / (math.sqrt( np.linalg.det(2.0 * math.pi * self.R) ))) * np.exp( -0.5 * ( (z-h).T.dot(self.Rinv.dot(z-h)) ) )
				# log likelihood
				#w = np.log(1.0 / (math.sqrt( np.linalg.det(2.0 * math.pi * self.R) ))) + ( -0.5 * ( (z-h).T.dot(self.Rinv.dot(z-h)) ) )
				w = ( -0.5 * ( (z-h).T.dot(self.Rinv.dot(z-h)) ) )
			except:
				print("Error on calc inverse weight ******")
				w = 0.0
			
			# -------------------------- #
			# Calc weight of Coplanarity #
			# -------------------------- #
			# Generate uvw1 (time:t-1)
			uvf1 = np.array([keypoint.x1, -keypoint.y1, -self.focus]) # Camera coordinates -> Device coordinates
			rotX = Util.rotationMatrixX(X1.o[0])
			rotY = Util.rotationMatrixY(X1.o[1])
			rotZ = Util.rotationMatrixZ(X1.o[2])
			# uvw1 = R(z)R(y)R(x)uvf1
			uvw1 = np.dot(rotZ,np.dot(rotY,np.dot(rotX,uvf1)))
			uvw1 /= 100.0 # Adjust scale to decrease calculation error. This doesn't have an influence to estimation.
			# Generate uvw2 (time:t)
			uvf2 = np.array([keypoint.x, -keypoint.y, -self.focus]) # Camera coordinates -> Device coordinates
			rotX = Util.rotationMatrixX(X_.o[0])
			rotY = Util.rotationMatrixY(X_.o[1])
			rotZ = Util.rotationMatrixZ(X_.o[2])
			# uvw2 = R(z)R(y)R(x)uvf2
			uvw2 = np.dot(rotZ,np.dot(rotY,np.dot(rotX,uvf2)))
			uvw2 /= 100.0 # Adjust scale to decrease calculation error. This doesn't have an influence to estimation.
			# Generate coplanarity matrix
			coplanarity_matrix = np.array([xyz,uvw1,uvw2])
			# Calc determinant
			determinant = np.linalg.det(coplanarity_matrix)
			# Weight
			w_coplanarity = 0.0
			try:
				#w_coplanarity = (1.0 / (math.sqrt( 2.0 * math.pi * self.noise_coplanarity**2 ))) * np.exp((determinant**2) / (-2.0 * (self.noise_coplanarity**2)) )
				# log likelihood				
				#w_coplanarity = np.log(1.0 / (math.sqrt( 2.0 * math.pi * self.noise_coplanarity**2 ))) + ((determinant**2) / (-2.0 * (self.noise_coplanarity**2)) )				
				w_coplanarity = ((determinant**2) / (-2.0 * (self.noise_coplanarity**2)) )
			except:
				print("Error on calc coplanarity weight ******")
				w_coplanarity = 0.0
			
			# --------------------------- #
			# inverse depth * coplanarity #
			# --------------------------- #
			if(self.count == 0):
				#print(w),
				#print(w_coplanarity)
				pass
			
			w = w + w_coplanarity  # use inverse depth * coplanarity log likelihood
			#w = w_coplanarity      # use only coplanarity log likelihood
			#w = w                  # use only inverse depth log likelihood
			
			weights.append(w)
			
		# ----------------------------------- #
		# sum log likelihood of all keypoints #
		# ----------------------------------- #
		for i,w in enumerate(weights):
			if(i==0):
				weight = w
			else:
				weight += w
				
		# ----------------------------- #
		# Average of log likelihood sum #
		# ----------------------------- #
		weight /= float(len(weights))
		
		###############################
		#print("weight "+str(weight))
		if(self.count == 0):
			#print("weight "+str(weight))
			#print("first_ratio "+str(float(count_of_first_observation)/float(len(keypoints)))),
			#print("("+str(count_of_first_observation)+"/"+str(len(keypoints))+")")
			pass
		###########################
			
		# 前回ランドマークをすべて消す
		for key in prevLandmarkKeys:
			del X_.landmarks[key]
			
		self.count+=1
		
		return X_, weight


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
		
		
	def reduce_particle_variance(self, X, X1, dt_camera):
		"""
		This method is called when No-resampling = True.
		Reduce particle variance to avoid divergence of particles.
		"""
		
		x = []
		o = []
		# Calc average
		for X_ in X:
			x.append(X_.x)
			o.append(X_.o)
		average = np.mean(x, axis=0)
		average_o = np.mean(o, axis=0)
		
		# Reduce variance
		for X_ in X:
			difference = X_.x - average
			X_.x = average + difference * 0.25
			X_.v = (X_.x - X1.x)/dt_camera

			#difference_o = X_.o - average_o
			X_.o = average_o
			
		return X


	def pf_step_camera_firsttime(self, X, dt, keypoints, step, P, M):
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

		
		# 推定と更新 prediction and update
		self.count=0
		
		X_predicted = range(M)
		weight = range(M)
		
		dt2 = 0.5*dt*dt
		
		for i in xrange(M):
			X_predicted[i], weight[i] = self.predictionAndUpdateOneParticle_firsttime(X[i], dt, dt2, keypoints, step, P)
		
		return X_predicted


	def pf_step_camera(self, X, dt, keypoints, step, P, M, X1, P1, dt_camera, gyro):
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
		self.count = 0
		no_resampling = False
		
		weight = range(M)
		log_likelihood = range(M)
		log_likelihood_max = 0.0
		X_resampled = range(M)
		X_predicted = range(M)
		
		dt2 = 0.5*dt*dt

		# 角度のノイズ　noise of orientation
		noise_o = gyro[2] * 0.1
		
		if(noise_o < 0.000001):
			noise_o = 0.000001
		
		# 推定と対数尤度計算 prediction and calc log likelihood
		for i in xrange(M):
			X_predicted[i], log_likelihood[i] = self.predictionAndUpdateOneParticle(X[i], dt, dt2, np.random.normal(0, self.noise_x_sys, 3), keypoints, step, P, X1, P1, dt_camera, np.array([0.0, 0.0, np.random.normal(0, noise_o)]))
			# 一番大きい対数尤度を記憶しておく
			if(i == 0):
				log_likelihood_max = log_likelihood[i]
			else:
				if(log_likelihood[i] > log_likelihood_max):
					log_likelihood_max = log_likelihood[i]
			
		# 対数尤度から重みを算出 log likelihood -> weight
		for i in xrange(M):
			weight[i] = np.exp(log_likelihood[i] - log_likelihood_max)
		
		# 正規化 normalization of weight
		try:
			weight_sum = sum(weight) # 総和 the sum of weights
			if(weight_sum > 1.01):
				# 重みの総和が大きい（尤度が高い）場合 likelihood is high enough
				#print("weight_sum "+str(weight_sum))     #########################
				# 正規化 normalization of weight
				weight = [(w/weight_sum) for w in weight]
				#for i in xrange(M):
				#	weight[i] /= weight_sum
				# リサンプリング re-sampling
				X_resampled = self.resampling(X_predicted, weight, M)
			else:
				# 重みの総和が小さい（尤度が低い）場合 likelihood is low
				#print("weight_sum "+str(weight_sum)),    #########################
				#print("************************************")
				# リサンプリングを行わない No re-sampling
				no_resampling = True
		except:
			print("Error on normalization of weight")
			# リサンプリングを行わない No re-sampling
			no_resampling = True
			
		if(no_resampling):
			#X_resampled = copy.deepcopy(X_predicted)
			X_resampled = self.reduce_particle_variance(X_predicted, X1, dt_camera)

		return X_resampled


	def pf_step_IMU(self, X, dt, accel, ori, isMoving, M, isFirstTimeCamera):
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

		#X_predicted = range(M)
		dt2 = 0.5*dt*dt
		
		# noise 
		length = np.linalg.norm(accel) * self.noise_x_sys_coefficient
		
		if(isFirstTimeCamera):
			length *= 0.1
		
		if(length < 0.0000001):
			length = 0.0000001

		"""
		direction = accel
		theta = math.atan2(direction[1],direction[0])
		phi = math.atan2(direction[2],math.hypot(direction[0],direction[1]))
		rotZ = Util.rotationMatrixZ(theta)
		rotY = Util.rotationMatrixY(-phi)
		rot = rotZ.dot(rotY)
		"""
		
		return [self.f_IMU(Xi, dt, dt2, accel, ori, isMoving, np.array([np.random.normal(0, length), np.random.normal(0, (length)), np.random.normal(0, (length))])) for Xi in X]
		
		#return [self.f_IMU(Xi, dt, dt2, accel, ori, isMoving, rot.dot(np.array([np.random.normal(0, length), np.random.normal(0, (length*0.4)), np.random.normal(0, (length*0.4))]))) for Xi in X]
		
		#return [self.f_IMU(Xi, dt, dt2, accel, ori, isMoving, np.random.normal(0, self.noise_x_sys, 3)) for Xi in X]