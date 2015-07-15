# -*- coding: utf-8 -*-

from math import *
import cv2 as cv
import numpy as np

def matrixGyro2Euler(x,y):
	#回転行列
	R = np.array([
		[0, sin(x)/cos(y), cos(x)/cos(y)],
		[0, cos(x), -sin(x)],
		[1, sin(x)*tan(y), cos(x)*tan(y)]
	])
	return R

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
