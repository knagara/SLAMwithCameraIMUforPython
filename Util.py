# -*- coding: utf-8 -*-

from math import *
import cv2 as cv
import numpy as np

def rotationMatrixX(r):
	C = cos(r)
	S = sin(r)
	#回転行列
	R = np.array([
		[1, 0, 0],
		[0, C, -S],
		[0, S, C]
	]) 
	return R
	
def rotationMatrixY(r):
	C = cos(r)
	S = sin(r)
	#回転行列
	R = np.array([
		[C, 0, S],
		[0, 1, 0],
		[-S, 0, C]
	]) 
	return R
	
def rotationMatrixZ(r):
	C = cos(r)
	S = sin(r)
	#回転行列
	R = np.array([
		[C, -S, 0],
		[S, C, 0],
		[0, 0, 1]
	]) 
	return R