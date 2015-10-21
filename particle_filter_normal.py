# -*- coding: utf-8 -*-
"""
Particle Filter
(IMU with PF model version)
"""

import numpy
import math

class ParticleFilterNormal:

	def __init__(self):
		pass
	

	def f(self, dt, x, u):
	    """ Transition model
	    - 状態方程式
	        x_t = f(x_t-1, u) + w
	        w ~ N(0, sigma)
	    """

	    return x
					

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
					

	def pf_step(self, X, u, y, N):
	    """One Step of Sampling Importance Resampling for Particle Filter
	    Parameters
	    ----------
	    X : array of [float|array]
	        状態 List of state set
	    u : float or array
	        制御入力 Control input
	    y : float or array
	        観測 Observation set
	    N : int
	        パーティクルの数 num of particles
	    Returns
	    -------
	    X_resampled : array of [float|array]
	        次の状態 List updated state
	    """

	    # 初期化
	    X_predicted = range(N)
	    weight = range(N)

	    for i in range(N):
	        # 推定 prediction
	        X_predicted[i] = self.f(X[i], u)
	        # 更新 update
	        weight[i] = self.likelihood(y, X_predicted[i])
	    # 正規化 normalization
	    weight_sum = sum(weight) # 総和 the sum of weights
	    for i in range(N):
	        weight[i] /= weight_sum
	    # リサンプリング re-sampling (if necessary)
	    X_resampled = self.resampling(X_predicted, weight, N)

	    return X_resampled
