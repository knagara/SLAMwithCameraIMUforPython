# -*- coding: utf-8 -*-

"""
landmark.py

author: Keita Nagara (University of Tokyo)

Class for landmark in 3D space
"""

from math import *
import numpy as np
import Util

class Landmark:

	def __init__(self, id_, step_, index_):
		self.id = id_
		self.step = step_
		self.index = index_
		self.mu = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
		self.sigma = np.zeros([])
		
	
	def init(self, X, keypoint, P, focus):
		self.mu[0] = X.x[0] # xi (Device position X at first observation)
		self.mu[1] = X.x[1] # yi (Device position Y at first observation)
		self.mu[2] = X.x[2] # zi (Device position Z at first observation)
		self.mu[3], self.mu[4] = self.initThetaPhi(X, keypoint, focus) # theta, phi (Azimuth & elevation of the ray at first observation)
		self.mu[5] = 0.1 # d_inv (Inverse depth at first observation. 0.1 means depth is 10 meter.)
		
		self.sigma = np.vstack((np.hstack((P,np.zeros([3,3]))),np.zeros([3,6])))
		self.sigma[3][3] = 0.01
		self.sigma[4][4] = 0.01
		self.sigma[5][5] = 0.5
		
	
	def initThetaPhi(self, X, keypoint, focus):
		uvf = np.array([keypoint.x, -keypoint.y, -focus]) # Camera coordinates -> Device coordinates
		rotX = Util.rotationMatrixX(X.o[0])
		rotY = Util.rotationMatrixY(X.o[1])
		rotZ = Util.rotationMatrixZ(X.o[2])
		# h = R(z)R(y)R(x)uvf
		h = np.dot(rotZ,np.dot(rotY,np.dot(rotX,uvf)))
		theta = atan2(h[0], h[2])
		phi = atan2(-h[1], hypot(h[0],h[2]))
		return theta, phi