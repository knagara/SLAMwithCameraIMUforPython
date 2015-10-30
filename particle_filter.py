# -*- coding: utf-8 -*-
"""
Particle Filter
This class is factory class.
"""

from particle_filter_coplanarity import ParticleFilterCoplanarity
from particle_filter_IMU import ParticleFilterIMU
from particle_filter_IMU2 import ParticleFilterIMU2

class ParticleFilter:

	def __init__(self):
		pass
		
	def getParticleFilterClass(self,stateType):
		if(stateType=="Coplanarity"):
			return ParticleFilterCoplanarity()
		elif(stateType=="IMUPF"):
			return ParticleFilterIMU()
		elif(stateType=="IMUPF2"):
			return ParticleFilterIMU2()
		else:
			pass
