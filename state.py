# -*- coding: utf-8 -*-

"""
state.py

author: Keita Nagara (University of Tokyo)

This class is called from "sensor.py" and "image.py", and estimate state variables.
This class is factory class. State type (& estimation model) is selected by argment.

"""

from state_IMU_KF import StateIMUKF

class State:

	def __init__(self,stateType):
		if(stateType=="IMUKF"):
			return StateIMUKF()
		elif(stateType=="IMUPF"):
			pass
		else:
			pass

