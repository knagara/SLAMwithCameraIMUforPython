# -*- coding: utf-8 -*-

"""
landmark.py

author: Keita Nagara　永良慶太 (University of Tokyo) <nagara.keita()gmail.com>

Class for landmark in 3D space
"""

from math import *
import numpy as np
import Util

class Landmark:

	def __init__(self):
		self.mu = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
		self.sigma = np.zeros([])
		self.cx = - 6.361694 # principal point X
		self.cy = - 22.962158 # principal point Y
		
	
	def init(self, X, keypoint, P, focus):
		self.mu[0] = X.x[0] # xi (Device position X at first observation)
		self.mu[1] = X.x[1] # yi (Device position Y at first observation)
		self.mu[2] = X.x[2] # zi (Device position Z at first observation)
		self.mu[3], self.mu[4] = self.initThetaPhi(X, keypoint, focus) # theta, phi (Azimuth & elevation of the ray at first observation)
		self.mu[5] = 0.2 # d_inv (Inverse depth at first observation. 0.2 means that depth is 5 meter.)
		
		self.sigma = np.vstack((np.hstack((P,np.zeros([3,3]))),np.zeros([3,6])))
		self.sigma[3][3] = 0.01
		self.sigma[4][4] = 0.01
		self.sigma[5][5] = 0.25
		
	
	def initThetaPhi(self, X, keypoint, focus):
		uvf = np.array([keypoint.x/focus, -keypoint.y/focus, -1]) # Camera coordinates -> Device coordinates, and normalized
		
		# Rotation matrix (Local coordinates -> Global coordinates)
		rotX = Util.rotationMatrixX(X.o[0])
		rotY = Util.rotationMatrixY(X.o[1])
		rotZ = Util.rotationMatrixZ(X.o[2])
		# h = R(z)R(y)R(x)uvf
		h = np.dot(rotZ,np.dot(rotY,np.dot(rotX,uvf)))
		theta = atan2(h[0], h[2])
		phi = atan2(-h[1], hypot(h[0],h[2]))
		return theta, phi
		
	
	def initPrev(self, X1, keypoint, P1, focus):
		self.mu[0] = X1.x[0] # xi (Device position X at first observation)
		self.mu[1] = X1.x[1] # yi (Device position Y at first observation)
		self.mu[2] = X1.x[2] # zi (Device position Z at first observation)
		self.mu[3], self.mu[4] = self.initThetaPhiPrev(X1, keypoint, focus) # theta, phi (Azimuth & elevation of the ray at first observation)
		self.mu[5] = 0.2 # d_inv (Inverse depth at first observation. 0.2 means that depth is 5 meter.)
		
		self.sigma = np.vstack((np.hstack((P1,np.zeros([3,3]))),np.zeros([3,6])))
		self.sigma[3][3] = 0.01
		self.sigma[4][4] = 0.01
		self.sigma[5][5] = 0.25
		
	
	def initThetaPhiPrev(self, X1, keypoint, focus):
		uvf = np.array([keypoint.x1/focus, -keypoint.y1/focus, -1]) # Camera coordinates -> Device coordinates, and normalized
		
		# Rotation matrix (Local coordinates -> Global coordinates)
		rotX = Util.rotationMatrixX(X1.o[0])
		rotY = Util.rotationMatrixY(X1.o[1])
		rotZ = Util.rotationMatrixZ(X1.o[2])
		# h = R(z)R(y)R(x)uvf
		h = np.dot(rotZ,np.dot(rotY,np.dot(rotX,uvf)))
		theta = atan2(h[0], h[2])
		phi = atan2(-h[1], hypot(h[0],h[2]))
		return theta, phi
		
		
	def getXYZ(self):
		# xi, yi, zi, xt, yt, zt, p (Inverse depth)
		xi = self.mu[0]
		yi = self.mu[1]
		zi = self.mu[2]
		p = self.mu[5]
		# sin, cos
		sinTheta = sin(self.mu[3])
		cosTheta = cos(self.mu[3])
		sinPhi = sin(self.mu[4])
		cosPhi = cos(self.mu[4])
		# XYZ = landmark position in XYZ
		XYZ = np.array([xi + (cosPhi * sinTheta)/p,
					yi - sinPhi/p,
					zi + (cosPhi * cosTheta)/p])
		return XYZ
		
		
		
	def h(self, position, o, focus):
		
		# often used variables
		# xi, yi, zi, xt, yt, zt, p (Inverse depth)
		xi = self.mu[0]
		yi = self.mu[1]
		zi = self.mu[2]
		xt = position[0]
		yt = position[1]
		zt = position[2]
		p = self.mu[5]
		# sin, cos
		sinTheta = sin(self.mu[3])
		cosTheta = cos(self.mu[3])
		sinPhi = sin(self.mu[4])
		cosPhi = cos(self.mu[4])
		
		# Rotation matrix (Global coordinates -> Local coordinates)
		rotXinv = Util.rotationMatrixX(-o[0])
		rotYinv = Util.rotationMatrixY(-o[1])
		rotZinv = Util.rotationMatrixZ(-o[2])
		R = np.dot(rotXinv, np.dot(rotYinv, rotZinv))
		
		# hG = [hx, hy, hz].T in the global coordinates
		hG = np.array([p * (xi - xt) + cosPhi * sinTheta,
					p * (yi - yt) - sinPhi,
					p * (zi - zt) + cosPhi * cosTheta])
					
		# XYZ = landmark position in XYZ
		XYZ = np.array([xi + (cosPhi * sinTheta)/p,
					yi - sinPhi/p,
					zi + (cosPhi * cosTheta)/p])
		
		# hL = h Local = [hx, hy, hz].T in the local coordinates
		hL = np.dot(R, hG)
		hx = hL[0]
		hy = hL[1]
		hz = hL[2]
		
		# h1 = - f*hx/hz, h2 = - f*hy/hz , and Device coordinates -> Camera coordinates
		h1 = - (focus * hx / hz)
		h2 = focus * hy / hz
		
		return np.array([h1,h2]), XYZ


	def calcObservation(self, X, focus):
		"""
		Calc h and H (Jacobian matrix of h)
		
		Observation function
			z = h(x) + v
			
			h(x) = [h1(x), h2(x)].T
			h1(x) = f*hx/hz - cx
			h2(x) = f*hy/hz - cy
		"""
		
		# often used variables
		# xi, yi, zi, xt, yt, zt, p (Inverse depth)
		xi = self.mu[0]
		yi = self.mu[1]
		zi = self.mu[2]
		xt = X.x[0]
		yt = X.x[1]
		zt = X.x[2]
		p = self.mu[5]
		# sin, cos
		sinTheta = sin(self.mu[3])
		cosTheta = cos(self.mu[3])
		sinPhi = sin(self.mu[4])
		cosPhi = cos(self.mu[4])
		
		# Rotation matrix (Global coordinates -> Local coordinates)
		rotXinv = Util.rotationMatrixX(-X.o[0])
		rotYinv = Util.rotationMatrixY(-X.o[1])
		rotZinv = Util.rotationMatrixZ(-X.o[2])
		R = np.dot(rotXinv, np.dot(rotYinv, rotZinv))
		
		# hG = [hx, hy, hz].T in the global coordinates
		hG = np.array([p * (xi - xt) + cosPhi * sinTheta,
					p * (yi - yt) - sinPhi,
					p * (zi - zt) + cosPhi * cosTheta])
		
		# hL = h Local = [hx, hy, hz].T in the local coordinates
		hL = np.dot(R, hG)
		hx = hL[0]
		hy = hL[1]
		hz = hL[2]
		
		# h1 = - f*hx/hz, h2 = - f*hy/hz , and Device coordinates -> Camera coordinates
		h1 = - (focus * hx / hz)
		h2 = focus * hy / hz
		
		# derivative
		R11 = R[0][0]
		R12 = R[0][1]
		R13 = R[0][2]
		R21 = R[1][0]
		R22 = R[1][1]
		R23 = R[1][2]
		R31 = R[2][0]
		R32 = R[2][1]
		R33 = R[2][2]
		
		dhxxi = p * R11
		dhyxi = p * R21
		dhzxi = p * R31
		
		dhxyi = p * R12
		dhyyi = p * R22
		dhzyi = p * R32
		
		dhxzi = p * R13
		dhyzi = p * R23
		dhzzi = p * R33
		
		dhxTheta = R11 * cosPhi * cosTheta - R13 * cosPhi * sinTheta
		dhyTheta = R21 * cosPhi * cosTheta - R23 * cosPhi * sinTheta
		dhzTheta = R31 * cosPhi * cosTheta - R33 * cosPhi * sinTheta
		
		dhxPhi = - R11 * sinTheta * sinPhi - R12 * cosPhi - R13 * cosTheta * sinPhi
		dhyPhi = - R21 * sinTheta * sinPhi - R22 * cosPhi - R23 * cosTheta * sinPhi
		dhzPhi = - R31 * sinTheta * sinPhi - R32 * cosPhi - R33 * cosTheta * sinPhi
		
		dhxp = R11 * (xi - xt) + R12 * (yi - yt) + R13 * (zi - zt)
		dhyp = R21 * (xi - xt) + R22 * (yi - yt) + R23 * (zi - zt)
		dhzp = R31 * (xi - xt) + R32 * (yi - yt) + R33 * (zi - zt)
		
		# Jacobian
		f_hz2 = focus / (hz * hz) # focus / (hz)^2
		
		dh1xi = - f_hz2 * (dhxxi * hz - hx * dhzxi)
		dh1yi = - f_hz2 * (dhxyi * hz - hx * dhzyi)
		dh1zi = - f_hz2 * (dhxzi * hz - hx * dhzzi)
		dh1Theta = - f_hz2 * (dhxTheta * hz - hx * dhzTheta)
		dh1Phi = - f_hz2 * (dhxPhi * hz - hx * dhzPhi)
		dh1p = - f_hz2 * (dhxp * hz - hx * dhzp)
		
		dh2xi = f_hz2 * (dhyxi * hz - hy * dhzxi)
		dh2yi = f_hz2 * (dhyyi * hz - hy * dhzyi)
		dh2zi = f_hz2 * (dhyzi * hz - hy * dhzzi)
		dh2Theta = f_hz2 * (dhyTheta * hz - hy * dhzTheta)
		dh2Phi = f_hz2 * (dhyPhi * hz - hy * dhzPhi)
		dh2p = f_hz2 * (dhyp * hz - hy * dhzp)
		
		Hm = np.array([[dh1xi, dh1yi, dh1zi, dh1Theta, dh1Phi, dh1p],
					[dh2xi, dh2yi, dh2zi, dh2Theta, dh2Phi, dh2p]])
		
		Hx = np.array([[-dh1xi, -dh1yi, -dh1zi],
					[-dh2xi, -dh2yi, -dh2zi]])
		
		return np.array([h1,h2]), Hx, Hm	
		