# -*- coding: utf-8 -*-

"""
state.py
Class for state of device

variables:
	t, t1, t2
	x, x1
	v, v1
	a, a1

methods:
	getter
	setter
	localization()
	init()

"""

import sys
from math import *
import cv2 as cv
import numpy as np
import KF


class State:

	def __init__(self):
		self.t = 0
		self.t1 = 0
		self.t2 = 0
		self.t3 = 0
		self.x = np.array([0.0,0.0,0.0])
		self.x1 = np.array([0.0,0.0,0.0])
		self.v = np.array([0.0,0.0,0.0])
		self.v1 = np.array([0.0,0.0,0.0])
		self.a = np.array([0.0,0.0,0.0])
		self.a1 = np.array([0.0,0.0,0.0])
		self.orientation = np.array([0.0,0.0,0.0])


	def init(self):
		self.t = 0
		self.t1 = 0
		self.t2 = 0
		self.t3 = 0
		self.x = np.array([0.0,0.0,0.0])
		self.x1 = np.array([0.0,0.0,0.0])
		self.v = np.array([0.0,0.0,0.0])
		self.v1 = np.array([0.0,0.0,0.0])
		self.a = np.array([0.0,0.0,0.0])
		self.a1 = np.array([0.0,0.0,0.0])
		self.orientation = np.array([0.0,0.0,0.0])


	def setTime(self,time):
		self.t3 = self.t2
		self.t2 = self.t1
		self.t1 = self.t
		self.t = time

	def setAccel(self,accel):
		self.a1 = self.a
		self.a = accel

	def setOrientation(self,_orientation):
		self.orientation = _orientation

	def getPosition(self):
		return self.x

	def getVelocity(self):
		return self.v

	def getAcceleration(self):
		return self.a

	def getOrientation(self):
		return self.orientation

	def getTimeDelta(self):
		return (self.t - self.t1)


	#estimate position by simple Eq.
	def simpleLocalization(self):

		if(self.t2 == 0):
			pass
		elif(self.t3 == 0):
			self.v = (self.t - self.t1)*self.a1
		else:
			self.v1 = self.v
			self.v = self.v1 + (self.t - self.t1)*self.a1
			self.x1 = self.x
			self.x = self.x1 + (self.t - self.t1)*self.v1 + 0.5*(self.t - self.t1)*(self.t - self.t1)*self.a1

			#print(self.x)


	#Estimate position of device
	#return position(x,y,z)
	def localization(self):
		self.simpleLocalization()
		#KFLocalization()


"""
	#estimate position by KF (Kalman Filter)
	def KFLocalization(self):
		Y = np.mat([[self.x[0]],[self.x[1]],[self.x[2]],[self.v[0]],[self.v[1]],[self.v[2]],[self.a[0]],[self.a[1]],[self.a[2]]])
		mu0 = np.mat([[self.x[0]],[self.x[1]],[self.x[2]],[self.v[0]],[self.v[1]],[self.v[2]],[self.a[0]],[self.a[1]],[self.a[2]]])
		Sigma0 =
		A =
		B =
		C =
		Q =
		R =
		pos = KF.execKF(1,)
"""

