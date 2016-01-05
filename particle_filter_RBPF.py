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

	def setParameter(self, param1, param2, param3, param4, param5):
		self.noise_x_sys = param1 # system noise of position (SD)　位置のシステムノイズ（標準偏差）
		self.noise_a_sys = param2 # system noise of acceleration (SD)　加速度のシステムノイズ（標準偏差）
		self.noise_g_sys = param3 # system noise of orientation (SD)　角度のシステムノイズ（標準偏差）
		self.noise_camera = param4 # observation noise of camera (SD) カメラの観測ノイズ（標準偏差）
		self.noise_coplanarity = param5 # observation noise of coplanarity (SD) 共面条件の観測ノイズ（標準偏差）
				
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
		
		"""
		if(isMoving[0] == False):
			X.v[0] = 0.0
		if(isMoving[1] == False):
			X.v[1] = 0.0
		if(isMoving[2] == False):
			X.v[2] = 0.0
		"""
	
		X_new = Particle()
		X_new.landmarks = X.landmarks
	
		# Transition with noise (only x,v)		
		X_new.x = X.x + dt*X.v + dt2*X.a + noise
		X_new.v = X.v + dt*X.a
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
				########################################
				if(self.count == 0):
					#print(X_.landmarks[landmarkId].sigma)
					pass
				########################################
			else:
				print("Error on pf_step_camera_firsttime")
				
		self.count+=1
		
		return X_, weight
	

	def predictionAndUpdateOneParticle(self, X, dt, dt2, noise, keypoints, step, P, X1, P1, dt_camera):
		
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
		#X_.v = X.v + dt*X.a + noise*50
		X_.a = X.a
		X_.o = X.o
		
		# 速度調整 velocity adjustment
		#X_.v = ((X_.x - X1.x)/dt_camera)
		X_.v = ((X_.x - X1.x)/dt_camera)*0.25
		#X_.v = ((X_.x - X1.x)/dt_camera)*0.5
		"""
		if(abs(X_.x[0] - X1.x[0]) < 0.05):
			X_.v[0] = ((X_.x[0] - X1.x[0])/dt_camera)*0.75
		if(abs(X_.x[1] - X1.x[1]) < 0.05):
			X_.v[1] = ((X_.x[1] - X1.x[1])/dt_camera)*0.75
		if(abs(X_.x[2] - X1.x[2]) < 0.05):
			X_.v[2] = ((X_.x[2] - X1.x[2])/dt_camera)*0.75
		"""
		
		# 共面条件モデルのための計算 Calc for Coplanarity
		xyz = np.array([X_.x[0] - X1.x[0], X_.x[1] - X1.x[1], X_.x[2] - X1.x[2]])
		
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
				########################################
				if(self.count == 0):
					count_of_first_observation += 1
					pass
				########################################
			else:
				# Already observed
				X_.landmarks[landmarkId] = X_.landmarks[prevLandmarkId]
				del X_.landmarks[prevLandmarkId]
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
				w = (1.0 / (math.sqrt( np.linalg.det(2.0 * math.pi * self.R) ))) * np.exp( -0.5 * ( (z-h).T.dot(self.Rinv.dot(z-h)) ) )
				#w *= 1000000
				w *= 1000000
			except:
				print("Error on calc weight at predictionAndUpdateOneParticle: ")
			
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
			w_coplanarity = (1.0 / (math.sqrt( 2.0 * math.pi * self.noise_coplanarity**2 ))) * np.exp((determinant**2) / (-2.0 * (self.noise_coplanarity**2)) )
			w_coplanarity *= 10
			
			# ------------------------------------- #
			# weight of inverse depth * coplanarity #
			# ------------------------------------- #
			#w = w                    # inverse depth only
			#w = w_coplanarity        # coplanarity only
			w = w * w_coplanarity    # inverse depth * coplanarity
			
			weights.append(w)
			
			###############################
			if(self.count == 0):
				#print("z-h "),
				#print(z-h)
				#print("weight  of inverse "),
				#print(w)
				#print("weight coplanarity "),
				#print(w_coplanarity)
				pass
			###############################
			
		try:
			for i,w in enumerate(weights):
				if(i==0):
					weight = w
				else:
					weight *= w
		except:
			print("Error on weight *= w at predictionAndUpdateOneParticle")
			for i,w in enumerate(weights):
				if(i==0):
					weight = (w/100000)
				else:
					weight *= (w/100000)
		
		###############################
		#print("weight "+str(weight))
		if(self.count == 0):
			#print("weight "+str(weight))
			print("first_ratio "+str(float(count_of_first_observation)/float(len(keypoints)))),
			print("("+str(count_of_first_observation)+"/"+str(len(keypoints))+")")
			pass
		###########################
			
		self.count+=1
		
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
		# Calc average of position
		for X_ in X:
			x.append(X_.x)
		average = np.mean(x, axis=0)
		
		# Reduce variance of position
		for X_ in X:
			difference = X_.x - average
			X_.x = average + difference*0.2
			X_.v = (X_.x - X1.x)/dt_camera
			
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
		###########################
		self.count=0
		###########################
		
		X_predicted = range(M)
		weight = range(M)
		
		dt2 = 0.5*dt*dt
		
		for i in xrange(M):
			X_predicted[i], weight[i] = self.predictionAndUpdateOneParticle_firsttime(X[i], dt, dt2, keypoints, step, P)
			#X_predicted[i], weight[i] = self.predictionAndUpdateOneParticle2_firsttime(X[i], dt, dt2, keypoints, step, P)
		
		return X_predicted


	def pf_step_camera(self, X, dt, keypoints, step, P, M, X1, P1, dt_camera):
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
		X_resampled = range(M)

		
		# 推定と更新 prediction and update
		###########################
		self.count=0
		###########################
		
		X_predicted = range(M)
		weight = range(M)
		no_resampling = False
		
		dt2 = 0.5*dt*dt
		
		for i in xrange(M):
			X_predicted[i], weight[i] = self.predictionAndUpdateOneParticle(X[i], dt, dt2, np.random.normal(0, self.noise_x_sys, 3), keypoints, step, P, X1, P1, dt_camera)
			#X_predicted[i], weight[i] = self.predictionAndUpdateOneParticle2(X[i], dt, dt2, keypoints, step, P)
		
		# 正規化 normalization of weight
		try:
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
				print("************************************")
				# リサンプリングを行わない No re-sampling
				#X_resampled = copy.deepcopy(X_predicted)
				no_resampling = True
		except:
			print("Error on normalization of weight")
			# リサンプリングを行わない No re-sampling
			#X_resampled = copy.deepcopy(X_predicted)
			no_resampling = True
			
		if(no_resampling):
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

		X_predicted = range(M)
		dt2 = 0.5*dt*dt
		
		# noise 
		length = np.linalg.norm(accel) * 0.25
		
		if(isFirstTimeCamera):
			length *= 0.1
		
		#if(length < self.noise_x_sys):
		#	length = self.noise_x_sys

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
		
		"""
		for i in xrange(M):
			noise = np.array([np.random.normal(0, length), np.random.normal(0, (length*0.4)), np.random.normal(0, (length*0.4))])
			noise = rot.dot(noise)
			X_predicted[i] = self.f_IMU(X[i], dt, dt2, accel, ori, isMoving, noise)
		"""
		
		return X_predicted
		
		
		
		#return [self.f_IMU(Xi, dt, dt2, accel, ori, isMoving, np.random.normal(0, self.noise_x_sys, 3)) for Xi in X]
		
