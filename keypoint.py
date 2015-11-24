# -*- coding: utf-8 -*-

"""
keypoint.py

author: Keita Nagara (University of Tokyo)

Class for key point in image
"""
 
from descriptor import Descriptor

class KeyPoint:

	def __init__(self,data_):
		
		cx = 540.0 + 19.840576 # (image size X)/2 +  principal point X
		cy = 960.0 + 9.901855 # (image size Y)/2 +  principal point Y
		
		data = data_.split(':')
		
		self.prevIndex = int(data[0])
		self.index = int(data[1])
		self.x = float(data[2]) - cx
		self.y = float(data[3]) - cy
		self.descriptor = Descriptor(data[4])