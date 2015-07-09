# -*- coding: utf-8 -*-

import numpy as np
from math import *
import Util


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

X = Util.rotationMatrixX(radians(90))
Y = Util.rotationMatrixY(radians(90))
Z = Util.rotationMatrixZ(radians(-60))

rot = np.dot(Z,np.dot(Y,X))

vec = np.array([0,1,0])

print(rot)
print(np.dot(rot,vec))

rot = np.dot(X,np.dot(Y,Z))

print(rot)

print(np.dot(rot,vec))