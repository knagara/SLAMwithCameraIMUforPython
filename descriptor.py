# -*- coding: utf-8 -*-

"""
descriptor.py

author: Keita Nagara (University of Tokyo)

Class for descriptor in image
"""

class Descriptor:

	def __init__(self,data_):
		
		data__ = data_.split(',')
		self.data = []
		
		for d in data__:
			if(d != ''):
				self.data.append(int(d))
		
		
	def printData(self):
		
		for d in self.data:
			print(d),

		print(" ")