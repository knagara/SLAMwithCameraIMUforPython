# -*- coding: utf-8 -*-
"""Particle Filter
"""

import time
import numpy
import matplotlib.pyplot as plt


class ParticleFilter:

	def __init__(self):
		pass

	def f(self, x, u):
	    """Transition model
	    - 状態方程式
	        f(x, u) = A * x + B * u + w
	        w ~ N(0, 2I)
	    - AとBは単位ベクトル
	    """
	    # noise sigma=2
	    w_mean = numpy.zeros(2)
	    w_cov = numpy.eye(2) * 2.
	    w = numpy.random.multivariate_normal(w_mean, w_cov)

	    x_transition = numpy.dot(numpy.diag([1., 1.]), x) + numpy.dot(numpy.diag([1., 1.]), u) + w

	    return x_transition

	def g(self, x):
	    """Obersvation model
	    - 観測方程式
	        g(x) = [|x-p1|, |x-p2|, |x-p3|, |x-p4|].T + v
	        v ~ N(0, 4I)
	    - ある4点からの距離
	    """
	    # observation points
	    p1 = [0., 0.]; p2 = [10., 0.]; p3 = [0., 10.]; p4 = [10., 10.]

	    # noise sigma=4
	    v_mean = numpy.zeros(4)
	    v_cov = numpy.eye(4) * 4.
	    v = numpy.random.multivariate_normal(v_mean, v_cov)

	    # observation vector
	    y = numpy.array([numpy.linalg.norm(x - p1),
	                     numpy.linalg.norm(x - p2),
	                     numpy.linalg.norm(x - p3),
	                     numpy.linalg.norm(x - p4)]) + v

	    return y

	def likelihood(self, y, x):
	    """Likelihood function
	    - 尤度関数
	        p(y|x) ~ exp(|y-g_(x)|**2 / simga**2)
	    - g_は誤差なしの観測方程式とする
	    - v ~ N(0, 4I)としたのでsigma**2=4
	    - 尤度 = 推定値と観測値との類似度
	    - アプリケーションによって適切に決めないといけない
	    - 物体追跡だと色情報を使ったりするかも
	    """
	    p1 = [0., 0.]; p2 = [10., 0.]; p3 = [0., 10.]; p4 = [10., 10.]
	    g_ = lambda x: numpy.array([numpy.linalg.norm(x - p1), numpy.linalg.norm(x - p2), numpy.linalg.norm(x - p3), numpy.linalg.norm(x - p4)])
	    return numpy.exp(-numpy.dot(y - g_(x), y - g_(x)) / 4)

	def resampling(self, X, W, M):
	    """Resampling
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
