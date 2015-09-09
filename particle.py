# -*- coding: utf-8 -*-

"""
particle.py

author: Keita Nagara (University of Tokyo)

Class for particle
"""

import numpy as np

class Particle:

	def __init__(self,accel,ori):

		self.x = np.array([0.0,0.0,0.0])
		self.v = np.array([0.0,0.0,0.0])
		self.a = accel
		self.o = ori

		self.keypoint = []
