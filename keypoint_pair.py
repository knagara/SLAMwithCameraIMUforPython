# -*- coding: utf-8 -*-

"""
keypoint.py

author: Keita Nagara (University of Tokyo)

Class for key points pair between images
"""

class KeyPointPair:

	def __init__(self,data_):
		
		data = data_.split(':')
		
		self.prevId = int(data[0])
		self.id = int(data[1])
		self.x1 = float(data[2])
		self.y1 = float(data[3])
		self.x2 = float(data[4])
		self.y2 = float(data[5])