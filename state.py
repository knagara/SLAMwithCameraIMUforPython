# -*- coding: utf-8 -*-

"""
state.py

author: Keita Nagara (University of Tokyo)

This class is called from "sensor.py" and "image.py", and estimate state variables.
This class is factory class. State type (& estimation model) is selected by argment.

"""

from state_IMU_KF import StateIMUKF
from state_IMU_PF import StateIMUPF

class State:

	def __init__(self):
		pass
	
	def getStateClass(self,stateType):
		if(stateType=="IMUKF"):
			return StateIMUKF()
		elif(stateType=="IMUPF"):
			state = StateIMUPF()
			state.initWithType("IMUPF")
			return state
		elif(stateType=="IMUPF2"):
			state = StateIMUPF()
			state.initWithType("IMUPF2")
			return state
		else:
			pass
