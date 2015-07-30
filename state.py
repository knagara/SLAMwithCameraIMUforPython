# -*- coding: utf-8 -*-

"""
state.py

author: Keita Nagara (University of Tokyo)

variables:
	t, t1, t2, t3
	x, x1
	v, v1
	a, a1
	orientation

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
		self.a2 = np.array([0.0,0.0,0.0])
		self.a3 = np.array([0.0,0.0,0.0])
		self.a4 = np.array([0.0,0.0,0.0])
		self.orientation = np.array([0.0,0.0,0.0])
		self.threshold = 0.05


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
		self.a2 = np.array([0.0,0.0,0.0])
		self.a3 = np.array([0.0,0.0,0.0])
		self.a4 = np.array([0.0,0.0,0.0])
		self.orientation = np.array([0.0,0.0,0.0])
		self.threshold = 0.05


	def setTime(self,time):
		self.t3 = self.t2
		self.t2 = self.t1
		self.t1 = self.t
		self.t = time

	def setAccel(self,accel):
		self.a4 = self.a3
		self.a3 = self.a2
		self.a2 = self.a1
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

		dt = self.t - self.t1
		
		if(self.t2 == 0):
			pass
		elif(self.t3 == 0):
			self.v = dt*self.a1
		else:
			self.v1 = self.v
			self.x1 = self.x
			
			#加速度がしきい値を下回る場合，速度と加速度をゼロとみなす
			# t-1 ～ t-4 まで判定し，すべて下回る場合のみ実行
			if(self.a1[0] < self.threshold and self.a1[0] > -self.threshold and self.a2[0] < self.threshold and self.a2[0] > -self.threshold and self.a3[0] < self.threshold and self.a3[0] > -self.threshold and self.a4[0] < self.threshold and self.a4[0] > -self.threshold):
				self.v1[0] = 0.0
				self.a1[0] = 0.0
			if(self.a1[1] < self.threshold and self.a1[1] > -self.threshold and self.a2[1] < self.threshold and self.a2[1] > -self.threshold and self.a3[2] < self.threshold and self.a3[2] > -self.threshold and self.a4[2] < self.threshold and self.a4[2] > -self.threshold):
				self.v1[1] = 0.0
				self.a1[1] = 0.0
			if(self.a1[2] < self.threshold and self.a1[2] > -self.threshold and self.a2[2] < self.threshold and self.a2[2] > -self.threshold and self.a3[2] < self.threshold and self.a3[2] > -self.threshold and self.a4[2] < self.threshold and self.a4[2] > -self.threshold):
				self.v1[2] = 0.0
				self.a1[2] = 0.0
				
			self.v = self.v1 + dt*self.a1
			self.x = self.x1 + dt*self.v1 + 0.5*dt*dt*self.a1
				


	#Estimate position of device
	#return position(x,y,z)
	def localization(self):
		self.simpleLocalization()



