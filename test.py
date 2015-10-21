# -*- coding: utf-8 -*-

import numpy as np
from math import *
import Util

X = Util.rotationMatrixX(radians(135))
Y = Util.rotationMatrixY(radians(0))
Z = Util.rotationMatrixZ(radians(0))

rot = np.dot(Z,np.dot(Y,X))

vec = np.array([0,1,1])

print(rot)
print(np.dot(rot,vec))

"""
rot = np.dot(X,np.dot(Y,Z))

print(rot)
print(np.dot(rot,vec))
"""
