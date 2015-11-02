# -*- coding: utf-8 -*-

"""
keypoint.py

author: Keita Nagara (University of Tokyo)

Class for key points pair between images
"""

class KeyPointPair:

	def __init__(self,data_):
		
		cx = 540.0 + 19.840576 # (image size X)/2 +  principal point X
		cy = 960.0 + 9.901855 # (image size Y)/2 +  principal point Y
		
		data = data_.split(':')
		
		self.prevId = int(data[0])
		self.id = int(data[1])
		self.x1 = float(data[2]) - cx
		self.y1 = float(data[3]) - cy
		self.x2 = float(data[4]) - cx
		self.y2 = float(data[5]) - cy