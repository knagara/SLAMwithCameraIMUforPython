# -*- coding: utf-8 -*-

"""
keypoint.py

author: Keita Nagara (University of Tokyo)

Class for key point in image
"""
 
from descriptor import Descriptor

class KeyPoint:

	def __init__(self,data_):
		
		data = data_.split(':')
		
		self.prevId = int(data[0])
		self.id = int(data[1])
		self.x = float(data[2])
		self.y = float(data[3])
		self.descriptor = Descriptor(data[4])