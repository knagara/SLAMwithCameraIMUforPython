# -*- coding: utf-8 -*-
"""
Image
This class is factory class.
"""

from image_coplanarity import ImageCoplanarity
from image_RBPF import ImageRBPF

class Image:

	def __init__(self):
		pass
		
	def getImageClass(self,stateType):
		if(stateType=="Coplanarity"):
			return ImageCoplanarity()
		elif(stateType=="RBPF"):
			return ImageRBPF()
		else:
			pass
