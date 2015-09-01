# -*- coding: utf-8 -*-

"""
image.py

author: Keita Nagara (University of Tokyo)

This class is called from "Main.py", and process image data.

methods:
	processData(data) <- called from "Main.py"
	
	init()
"""

import sys
from math import *
import cv2 as cv
import numpy as np
import Util
from keypoint import KeyPoint

class Image:

	def __init__(self,_state):
		#state.py
		self.state = _state
		#variables
		


	def init(self):
		#state.py
		#self.state.init() #this is also called in sensor.py
		#variables
		pass



	#Set new data and Execute all functions
	def processData(self,data):
		
		#if nomatch then nothing to do
		if(data[0] == "nomatch"):
			print("nomatch")
			return
		
		print("+"),

		keypoints = []
		for d in data:
			if(d != ''):
				keypoints.append(KeyPoint(d))
		
		#print(str(keypoints[0].x))
		#print(str(keypoints[0].y))
		#keypoints[0].descriptor.printData()
		
