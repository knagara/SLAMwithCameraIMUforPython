# -*- coding: utf-8 -*-

"""
particle.py

author: Keita Nagara (University of Tokyo)

Class for particle
"""

import numpy as np


class Particle:

	def __init__(self):

		self.x = np.array([0.0,0.0,0.0])
		self.v = np.array([0.0,0.0,0.0])
		self.a = np.array([0.0,0.0,0.0])
		self.o = np.array([0.0,0.0,0.0])

		self.landmarks = []
		
		
	def initWithMu(self,mu):
		self.x = np.array([mu[0],mu[1],mu[2]])
		self.o = np.array([mu[9],mu[10],mu[11]])
		
		
	def initWithIMU(self,accel,ori):
		self.a = accel
		self.o = ori
		
	
	def initWithStateVector(self, mu, sigma):
		#generate random state from mu. sigma
		state = np.random.multivariate_normal(mu, sigma)
		
		self.x = np.array([state[0],state[1],state[2]])
		self.v = np.array([state[3],state[4],state[5]])
		self.a = np.array([state[6],state[7],state[8]])
		self.o = np.array([state[9],state[10],state[11]])
		
	
	def findLandmark(self, step_, index_):
		if(step_ == -1):
			return "none"
			
		if(len(self.landmarks) == 0):
			return "none"
		
		for landmark in reversed(self.landmarks):
			if(landmark.step < step_):
				break
			if(landmark.step == step_ and landmark.index == index_):
				return landmark
			
		return "none"
