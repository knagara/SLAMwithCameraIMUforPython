# -*- coding: utf-8 -*-

"""
particle.py

author: Keita Nagara　永良慶太 (University of Tokyo) <nagara.keita()gmail.com>

Class for particle
"""

import numpy as np
import time


class Particle:

	def __init__(self):

		self.x = np.array([0.0,0.0,0.0])
		self.v = np.array([0.0,0.0,0.0])
		self.a = np.array([0.0,0.0,0.0])
		self.o = np.array([0.0,0.0,0.0])

		self.landmarks = {}
		
		
	def initWithPositionAndOrientation(self, x_, o_):
		self.x = x_
		self.o = o_
		
		
	def initWithMu(self,mu):
		self.x = np.array([mu[0],mu[1],mu[2]])
		self.o = np.array([mu[9],mu[10],mu[11]])
		
		
	def initWithIMU(self,accel,ori):
		self.a = accel
		self.o = ori
		
	
	def initWithStateVector(self, mu, sigma):
		#generate random state from mu. sigma
		try:
			state = np.random.multivariate_normal(mu, sigma)
		except:
			print("Error on initWithStateVector np.random.multivariate_normal(mu, sigma)")
			state = mu
		
		self.x = np.array([state[0],state[1],state[2]])
		self.v = np.array([state[3],state[4],state[5]])
		self.a = np.array([state[6],state[7],state[8]])
		self.o = np.array([state[9],state[10],state[11]])
		

	def appendLandmark(self, key, landmark):
		self.landmarks[key] = landmark	
		
	
	def printXYZ(self):
		print(str(self.x[0])+","+str(self.x[1])+","+str(self.x[2]))
		
		
		
		
