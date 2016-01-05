# -*- coding: utf-8 -*-

from math import *
import cv2 as cv
import numpy as np

def isDeviceMoving(a):
	if(abs(a) < 0.005):
		return False
	else:
		return True

def lowPassFilter(value,newValue,alpha):
	value = value * alpha + newValue * (1 - alpha)
	return value
	
"""
High-pass filter
 - newValues 新しい値
 - lowValue 前回の低周波領域値が渡され、今回の低周波領域値が格納される配列
 - value ハイパスフィルタ適用後の値が格納される配列
"""
def highPassFilter(newValues, lowValue, alpha):
	lowValue = alpha * lowValue + (1 - alpha) * newValues
	value = newValues - lowValue
	return value, lowValue

def matrixGyro2Euler(x,y):
	#回転行列
	R = np.array([
		[1, sin(x)*tan(y), cos(x)*tan(y)],
		[0, cos(x), -sin(x)],
		[0, sin(x)/cos(y), cos(x)/cos(y)]
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
