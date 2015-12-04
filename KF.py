# -*- coding: utf-8 -*-

import copy
import numpy as np


def execKF1Update(z, mu0, Sigma0, H, R):
    '''Linear Kalman Filter

    - 観測方程式
        z = H * x + v, v ~ N(0,R)

    Parameters
    ==========
    - z : 観測列
    - mu0 : 初期状態推定値
    - Sigma0 : 初期誤差共分散行列
    - H, R : カルマンフィルタの係数

    Returns
    =======
    - M : 状態推定値列
    '''

    mu = copy.deepcopy(mu0) # 初期状態推定値
    Sigma = copy.deepcopy(Sigma0) # 初期誤差共分散行列

    # 更新
    S = H.dot(Sigma.dot(H.T)) + R
    K = Sigma.dot(H.T.dot(np.linalg.inv(S)))
    mu = mu + K.dot(z - H.dot(mu))
    Sigma = Sigma - K.dot(H.dot(Sigma))

    return (mu, Sigma)


def execKF1Simple(Y, mu0, Sigma0, A, C, Q, R):
    '''Linear Kalman Filter

    - 状態方程式
        x = A * x_ + w, w ~ N(0,Q)
    - 観測方程式
        y = C * x + v, v ~ N(0,R)

    Parameters
    ==========
    - Y : 観測列
    - mu0 : 初期状態推定値
    - Sigma0 : 初期誤差共分散行列
    - A, C, Q, R : カルマンフィルタの係数

    Returns
    =======
    - M : 状態推定値列
    '''

    mu = copy.deepcopy(mu0) # 初期状態推定値
    Sigma = copy.deepcopy(Sigma0) # 初期誤差共分散行列

    # 推定
    mu_ = A.dot(mu)
    Sigma_ = Q + A.dot(Sigma.dot(A.T))

    # 更新
    yi = Y - C.dot(mu_)
    S = C.dot(Sigma_.dot(C.T)) + R
    K = Sigma_.dot(C.T.dot(np.linalg.inv(S)))
    mu = mu_ + K.dot(yi)
    Sigma = Sigma_ - K.dot(C.dot(Sigma_))

    return (mu, Sigma)


def execKF1(Y, U, mu0, Sigma0, A, B, C, Q, R):
    '''Linear Kalman Filter

    - 状態方程式
        x = A * x_ + B * u + w, w ~ N(0,Q)
    - 観測方程式
        y = C * x + v, v ~ N(0,R)

    Parameters
    ==========
    - Y : 観測列
    - U : 入力列
    - mu0 : 初期状態推定値
    - Sigma0 : 初期誤差共分散行列
    - A, B, C, Q, R : カルマンフィルタの係数

    Returns
    =======
    - M : 状態推定値列
    '''

    mu = mu0 # 初期状態推定値
    Sigma = Sigma0 # 初期誤差共分散行列

    # 推定
    mu_ = A.dot(mu) + B.dot(U)
    Sigma_ = Q + A.dot(Sigma.dot(A.T))

    # 更新
    yi = Y - C.dot(mu_)
    S = C.dot(Sigma_.dot(C.T)) + R
    K = Sigma_.dot(C.T.dot(np.linalg.inv(S)))
    mu = mu_ + K.dot(yi)
    Sigma = Sigma_ - K.dot(C.dot(Sigma_))

    return (mu, Sigma)


def execKF(T, Y, U, mu0, Sigma0, A, B, C, Q, R):
    '''Linear Kalman Filter

    - 状態方程式
        x = A * x_ + B * u + w, w ~ N(0,Q)
    - 観測方程式
        y = C * x + v, v ~ N(0,R)

    Parameters
    ==========
    - T : ステップ数
    - Y : 観測列
    - U : 入力列
    - mu0 : 初期状態推定値
    - Sigma0 : 初期誤差共分散行列
    - A, B, C, Q, R : カルマンフィルタの係数

    Returns
    =======
    - M : 状態推定値列
    '''

    mu = mu0 # 初期状態推定値
    Sigma = Sigma0 # 初期誤差共分散行列

    M = [mu] # 状態推定値列

    for i in range(T):
        # 推定
        mu_ = A * mu + B * U[i]
        Sigma_ = Q + A * Sigma * A.T

        # 更新
        yi = Y[i+1] - C * mu_
        S = C * Sigma_ * C.T + R
        K = Sigma_ * C.T * S.I
        mu = mu_ + K * yi
        Sigma = Sigma_ - K * C * Sigma_
        M.append(mu)

    return M
