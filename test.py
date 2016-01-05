# coding: utf-8
import numpy as np
import Util
from math import *

def main():
	
	# 画像サイズ
	width = 720
	height = 1280
	# 焦点距離
	f = 924.1770935
	# 主点位置
	cx = - 6.361694
	cy = - 22.962158
	
	# ランドマーク座標
	landmark1 = np.array([0.6, -5.0, -1.0])
	landmark2 = np.array([0.0, -5.0, 1.0])
	landmark3 = np.array([1.2, -5.0, 1.0])
	landmark4 = np.array([-0.2, -5.0, -0.0])
	landmark5 = np.array([1.4, -5.0, -0.0])
	
	landmark6 = np.array([0.6, -3.0, -0.5]) #途中から追加する
	landmark7 = np.array([0.6, -8.0, -0.5]) #途中から追加する
	
	# 1 座標
	x1 = np.array([0.0,0.0,0.0])
	
	# 1 回転角
	ox1 = - pi / 2.0
	oy1 = 0.0
	oz1 = 0.0
	
	# 1 回転行列 W -> C
	rotXinv = Util.rotationMatrixX(-ox1)
	rotYinv = Util.rotationMatrixY(-oy1)
	rotZinv = Util.rotationMatrixZ(-oz1)
	R1 = np.dot(rotXinv, np.dot(rotYinv, rotZinv))
	
	# 1 画像座標
	landmark1_1 = R1.dot(landmark1 - x1)
	u11 = - (f * landmark1_1[0] / landmark1_1[2]) + width/2.0 + cx
	v11 = f * landmark1_1[1] / landmark1_1[2] + height/2.0 + cy
	print("(u1,v1)1 = "+str(u11)+":"+str(v11))
	landmark2_1 = R1.dot(landmark2 - x1)
	u21 = - (f * landmark2_1[0] / landmark2_1[2]) + width/2.0 + cx
	v21 = f * landmark2_1[1] / landmark2_1[2] + height/2.0 + cy
	print("(u2,v2)1 = "+str(u21)+":"+str(v21))
	landmark3_1 = R1.dot(landmark3 - x1)
	u31 = - (f * landmark3_1[0] / landmark3_1[2]) + width/2.0 + cx
	v31 = f * landmark3_1[1] / landmark3_1[2] + height/2.0 + cy
	print("(u3,v3)1 = "+str(u31)+":"+str(v31))
	landmark4_1 = R1.dot(landmark4 - x1)
	u41 = - (f * landmark4_1[0] / landmark4_1[2]) + width/2.0 + cx
	v41 = f * landmark4_1[1] / landmark4_1[2] + height/2.0 + cy
	print("(u4,v4)1 = "+str(u41)+":"+str(v41))
	landmark5_1 = R1.dot(landmark5 - x1)
	u51 = - (f * landmark5_1[0] / landmark5_1[2]) + width/2.0 + cx
	v51 = f * landmark5_1[1] / landmark5_1[2] + height/2.0 + cy
	print("(u5,v5)1 = "+str(u51)+":"+str(v51))
	print(" ")
	
	# 2 座標
	x2 = np.array([0.11,-0.011,0.0])
	
	# 2 回転角
	ox = - pi / 2.0
	oy = 0.0
	oz = 0.01
	
	# 2 回転行列 W -> C
	rotXinv = Util.rotationMatrixX(-ox)
	rotYinv = Util.rotationMatrixY(-oy)
	rotZinv = Util.rotationMatrixZ(-oz)
	R = np.dot(rotXinv, np.dot(rotYinv, rotZinv))
	
	# 2 画像座標
	landmark1_1 = R.dot(landmark1 - x2)
	u12 = - (f * landmark1_1[0] / landmark1_1[2]) + width/2.0 + cx
	v12 = f * landmark1_1[1] / landmark1_1[2] + height/2.0 + cy
	print("(u1,v1)2 = "+str(u12)+":"+str(v12))
	landmark2_1 = R.dot(landmark2 - x2)
	u22 = - (f * landmark2_1[0] / landmark2_1[2]) + width/2.0 + cx
	v22 = f * landmark2_1[1] / landmark2_1[2] + height/2.0 + cy
	print("(u2,v2)2 = "+str(u22)+":"+str(v22))
	landmark3_1 = R.dot(landmark3 - x2)
	u32 = - (f * landmark3_1[0] / landmark3_1[2]) + width/2.0 + cx
	v32 = f * landmark3_1[1] / landmark3_1[2] + height/2.0 + cy
	print("(u3,v3)2 = "+str(u32)+":"+str(v32))
	landmark4_1 = R.dot(landmark4 - x2)
	u42 = - (f * landmark4_1[0] / landmark4_1[2]) + width/2.0 + cx
	v42 = f * landmark4_1[1] / landmark4_1[2] + height/2.0 + cy
	print("(u4,v4)2 = "+str(u42)+":"+str(v42))
	landmark5_1 = R.dot(landmark5 - x2)
	u52 = - (f * landmark5_1[0] / landmark5_1[2]) + width/2.0 + cx
	v52 = f * landmark5_1[1] / landmark5_1[2] + height/2.0 + cy
	print("(u5,v5)2 = "+str(u52)+":"+str(v52))
	landmark6_1 = R.dot(landmark6 - x2)
	u62 = - (f * landmark6_1[0] / landmark6_1[2]) + width/2.0 + cx
	v62 = f * landmark6_1[1] / landmark6_1[2] + height/2.0 + cy
	print("(u6,v6)2 = "+str(u62)+":"+str(v62))
	landmark7_1 = R.dot(landmark7 - x2)
	u72 = - (f * landmark7_1[0] / landmark7_1[2]) + width/2.0 + cx
	v72 = f * landmark7_1[1] / landmark7_1[2] + height/2.0 + cy
	print("(u7,v7)2 = "+str(u72)+":"+str(v72))
	print(" ")
	
	# 3 座標
	x3 = np.array([0.21,-0.021,0.0])
	
	# 3 回転角
	ox = - pi / 2.0
	oy = 0.0
	oz = 0.02
	
	# 3 回転行列 W -> C
	rotXinv = Util.rotationMatrixX(-ox)
	rotYinv = Util.rotationMatrixY(-oy)
	rotZinv = Util.rotationMatrixZ(-oz)
	R = np.dot(rotXinv, np.dot(rotYinv, rotZinv))
	
	# 3 画像座標
	landmark1_1 = R.dot(landmark1 - x3)
	u13 = - (f * landmark1_1[0] / landmark1_1[2]) + width/2.0 + cx
	v13 = f * landmark1_1[1] / landmark1_1[2] + height/2.0 + cy
	print("(u1,v1)3 = "+str(u13)+":"+str(v13))
	landmark2_1 = R.dot(landmark2 - x3)
	u23 = - (f * landmark2_1[0] / landmark2_1[2]) + width/2.0 + cx
	v23 = f * landmark2_1[1] / landmark2_1[2] + height/2.0 + cy
	print("(u2,v2)3 = "+str(u23)+":"+str(v23))
	landmark3_1 = R.dot(landmark3 - x3)
	u33 = - (f * landmark3_1[0] / landmark3_1[2]) + width/2.0 + cx
	v33 = f * landmark3_1[1] / landmark3_1[2] + height/2.0 + cy
	print("(u3,v3)3 = "+str(u33)+":"+str(v33))
	landmark4_1 = R.dot(landmark4 - x3)
	u43 = - (f * landmark4_1[0] / landmark4_1[2]) + width/2.0 + cx
	v43 = f * landmark4_1[1] / landmark4_1[2] + height/2.0 + cy
	print("(u4,v4)3 = "+str(u43)+":"+str(v43))
	landmark5_1 = R.dot(landmark5 - x3)
	u53 = - (f * landmark5_1[0] / landmark5_1[2]) + width/2.0 + cx
	v53 = f * landmark5_1[1] / landmark5_1[2] + height/2.0 + cy
	print("(u5,v5)3 = "+str(u53)+":"+str(v53))
	landmark6_1 = R.dot(landmark6 - x3)
	u63 = - (f * landmark6_1[0] / landmark6_1[2]) + width/2.0 + cx
	v63 = f * landmark6_1[1] / landmark6_1[2] + height/2.0 + cy
	print("(u6,v6)3 = "+str(u63)+":"+str(v63))
	landmark7_1 = R.dot(landmark7 - x3)
	u73 = - (f * landmark7_1[0] / landmark7_1[2]) + width/2.0 + cx
	v73 = f * landmark7_1[1] / landmark7_1[2] + height/2.0 + cy
	print("(u7,v7)3 = "+str(u73)+":"+str(v73))
	print(" ")
	
	# 4 座標
	x4 = np.array([0.31,-0.031,0.0])
	
	# 4 回転角
	ox = - pi / 2.0
	oy = 0.0
	oz = 0.03
	
	# 4 回転行列 W -> C
	rotXinv = Util.rotationMatrixX(-ox)
	rotYinv = Util.rotationMatrixY(-oy)
	rotZinv = Util.rotationMatrixZ(-oz)
	R = np.dot(rotXinv, np.dot(rotYinv, rotZinv))
	
	# 4 画像座標
	landmark1_1 = R.dot(landmark1 - x4)
	u14 = - (f * landmark1_1[0] / landmark1_1[2]) + width/2.0 + cx
	v14 = f * landmark1_1[1] / landmark1_1[2] + height/2.0 + cy
	print("(u1,v1)4 = "+str(u14)+":"+str(v14))
	landmark2_1 = R.dot(landmark2 - x4)
	u24 = - (f * landmark2_1[0] / landmark2_1[2]) + width/2.0 + cx
	v24 = f * landmark2_1[1] / landmark2_1[2] + height/2.0 + cy
	print("(u2,v2)4 = "+str(u24)+":"+str(v24))
	landmark3_1 = R.dot(landmark3 - x4)
	u34 = - (f * landmark3_1[0] / landmark3_1[2]) + width/2.0 + cx
	v34 = f * landmark3_1[1] / landmark3_1[2] + height/2.0 + cy
	print("(u3,v3)4 = "+str(u34)+":"+str(v34))
	landmark4_1 = R.dot(landmark4 - x4)
	u44 = - (f * landmark4_1[0] / landmark4_1[2]) + width/2.0 + cx
	v44 = f * landmark4_1[1] / landmark4_1[2] + height/2.0 + cy
	print("(u4,v4)4 = "+str(u44)+":"+str(v44))
	landmark5_1 = R.dot(landmark5 - x4)
	u54 = - (f * landmark5_1[0] / landmark5_1[2]) + width/2.0 + cx
	v54 = f * landmark5_1[1] / landmark5_1[2] + height/2.0 + cy
	print("(u5,v5)4 = "+str(u54)+":"+str(v54))
	landmark6_1 = R.dot(landmark6 - x4)
	u64 = - (f * landmark6_1[0] / landmark6_1[2]) + width/2.0 + cx
	v64 = f * landmark6_1[1] / landmark6_1[2] + height/2.0 + cy
	print("(u6,v6)4 = "+str(u64)+":"+str(v64))
	landmark7_1 = R.dot(landmark7 - x4)
	u74 = - (f * landmark7_1[0] / landmark7_1[2]) + width/2.0 + cx
	v74 = f * landmark7_1[1] / landmark7_1[2] + height/2.0 + cy
	print("(u7,v7)4 = "+str(u74)+":"+str(v74))
	print(" ")
	
	# 5 座標
	x5 = np.array([0.41,-0.041,0.0])
	
	# 5 回転角
	ox = - pi / 2.0
	oy = 0.0
	oz = 0.04
	
	# 5 回転行列 W -> C
	rotXinv = Util.rotationMatrixX(-ox)
	rotYinv = Util.rotationMatrixY(-oy)
	rotZinv = Util.rotationMatrixZ(-oz)
	R = np.dot(rotXinv, np.dot(rotYinv, rotZinv))
	
	# 5 画像座標
	landmark1_1 = R.dot(landmark1 - x5)
	u15 = - (f * landmark1_1[0] / landmark1_1[2]) + width/2.0 + cx
	v15 = f * landmark1_1[1] / landmark1_1[2] + height/2.0 + cy
	print("(u1,v1)5 = "+str(u15)+":"+str(v15))
	landmark2_1 = R.dot(landmark2 - x5)
	u25 = - (f * landmark2_1[0] / landmark2_1[2]) + width/2.0 + cx
	v25 = f * landmark2_1[1] / landmark2_1[2] + height/2.0 + cy
	print("(u2,v2)5 = "+str(u25)+":"+str(v25))
	landmark3_1 = R.dot(landmark3 - x5)
	u35 = - (f * landmark3_1[0] / landmark3_1[2]) + width/2.0 + cx
	v35 = f * landmark3_1[1] / landmark3_1[2] + height/2.0 + cy
	print("(u3,v3)5 = "+str(u35)+":"+str(v35))
	landmark4_1 = R.dot(landmark4 - x5)
	u45 = - (f * landmark4_1[0] / landmark4_1[2]) + width/2.0 + cx
	v45 = f * landmark4_1[1] / landmark4_1[2] + height/2.0 + cy
	print("(u4,v4)5 = "+str(u45)+":"+str(v45))
	landmark5_1 = R.dot(landmark5 - x5)
	u55 = - (f * landmark5_1[0] / landmark5_1[2]) + width/2.0 + cx
	v55 = f * landmark5_1[1] / landmark5_1[2] + height/2.0 + cy
	print("(u5,v5)5 = "+str(u55)+":"+str(v55))
	landmark6_1 = R.dot(landmark6 - x5)
	u65 = - (f * landmark6_1[0] / landmark6_1[2]) + width/2.0 + cx
	v65 = f * landmark6_1[1] / landmark6_1[2] + height/2.0 + cy
	print("(u6,v6)5 = "+str(u65)+":"+str(v65))
	landmark7_1 = R.dot(landmark7 - x5)
	u75 = - (f * landmark7_1[0] / landmark7_1[2]) + width/2.0 + cx
	v75 = f * landmark7_1[1] / landmark7_1[2] + height/2.0 + cy
	print("(u7,v7)5 = "+str(u75)+":"+str(v75))
	print(" ")
	
	# 6 座標
	x6 = np.array([0.51,-0.051,0.0])
	
	# 6 回転角
	ox = - pi / 2.0
	oy = 0.0
	oz = 0.05
	
	# 6 回転行列 W -> C
	rotXinv = Util.rotationMatrixX(-ox)
	rotYinv = Util.rotationMatrixY(-oy)
	rotZinv = Util.rotationMatrixZ(-oz)
	R = np.dot(rotXinv, np.dot(rotYinv, rotZinv))
	
	# 6 画像座標
	landmark1_1 = R.dot(landmark1 - x6)
	u16 = - (f * landmark1_1[0] / landmark1_1[2]) + width/2.0 + cx
	v16 = f * landmark1_1[1] / landmark1_1[2] + height/2.0 + cy
	print("(u1,v1)6 = "+str(u16)+":"+str(v16))
	landmark2_1 = R.dot(landmark2 - x6)
	u26 = - (f * landmark2_1[0] / landmark2_1[2]) + width/2.0 + cx
	v26 = f * landmark2_1[1] / landmark2_1[2] + height/2.0 + cy
	print("(u2,v2)6 = "+str(u26)+":"+str(v26))
	landmark3_1 = R.dot(landmark3 - x6)
	u36 = - (f * landmark3_1[0] / landmark3_1[2]) + width/2.0 + cx
	v36 = f * landmark3_1[1] / landmark3_1[2] + height/2.0 + cy
	print("(u3,v3)6 = "+str(u36)+":"+str(v36))
	landmark4_1 = R.dot(landmark4 - x6)
	u46 = - (f * landmark4_1[0] / landmark4_1[2]) + width/2.0 + cx
	v46 = f * landmark4_1[1] / landmark4_1[2] + height/2.0 + cy
	print("(u4,v4)6 = "+str(u46)+":"+str(v46))
	landmark5_1 = R.dot(landmark5 - x6)
	u56 = - (f * landmark5_1[0] / landmark5_1[2]) + width/2.0 + cx
	v56 = f * landmark5_1[1] / landmark5_1[2] + height/2.0 + cy
	print("(u5,v5)6 = "+str(u56)+":"+str(v56))
	landmark6_1 = R.dot(landmark6 - x6)
	u66 = - (f * landmark6_1[0] / landmark6_1[2]) + width/2.0 + cx
	v66 = f * landmark6_1[1] / landmark6_1[2] + height/2.0 + cy
	print("(u6,v6)6 = "+str(u66)+":"+str(v66))
	landmark7_1 = R.dot(landmark7 - x6)
	u76 = - (f * landmark7_1[0] / landmark7_1[2]) + width/2.0 + cx
	v76 = f * landmark7_1[1] / landmark7_1[2] + height/2.0 + cy
	print("(u7,v7)6 = "+str(u76)+":"+str(v76))
	print(" ")
	
	# 7 座標
	x7 = np.array([0.61,-0.061,0.0])
	
	# 7 回転角
	ox = - pi / 2.0
	oy = 0.0
	oz = 0.06
	
	# 7 回転行列 W -> C
	rotXinv = Util.rotationMatrixX(-ox)
	rotYinv = Util.rotationMatrixY(-oy)
	rotZinv = Util.rotationMatrixZ(-oz)
	R = np.dot(rotXinv, np.dot(rotYinv, rotZinv))
	
	# 7 画像座標
	landmark1_1 = R.dot(landmark1 - x7)
	u17 = - (f * landmark1_1[0] / landmark1_1[2]) + width/2.0 + cx
	v17 = f * landmark1_1[1] / landmark1_1[2] + height/2.0 + cy
	print("(u1,v1)7 = "+str(u17)+":"+str(v17))
	landmark2_1 = R.dot(landmark2 - x7)
	u27 = - (f * landmark2_1[0] / landmark2_1[2]) + width/2.0 + cx
	v27 = f * landmark2_1[1] / landmark2_1[2] + height/2.0 + cy
	print("(u2,v2)7 = "+str(u27)+":"+str(v27))
	landmark3_1 = R.dot(landmark3 - x7)
	u37 = - (f * landmark3_1[0] / landmark3_1[2]) + width/2.0 + cx
	v37 = f * landmark3_1[1] / landmark3_1[2] + height/2.0 + cy
	print("(u3,v3)7 = "+str(u37)+":"+str(v37))
	landmark4_1 = R.dot(landmark4 - x7)
	u47 = - (f * landmark4_1[0] / landmark4_1[2]) + width/2.0 + cx
	v47 = f * landmark4_1[1] / landmark4_1[2] + height/2.0 + cy
	print("(u4,v4)7 = "+str(u47)+":"+str(v47))
	landmark5_1 = R.dot(landmark5 - x7)
	u57 = - (f * landmark5_1[0] / landmark5_1[2]) + width/2.0 + cx
	v57 = f * landmark5_1[1] / landmark5_1[2] + height/2.0 + cy
	print("(u5,v5)7 = "+str(u57)+":"+str(v57))
	landmark6_1 = R.dot(landmark6 - x7)
	u67 = - (f * landmark6_1[0] / landmark6_1[2]) + width/2.0 + cx
	v67 = f * landmark6_1[1] / landmark6_1[2] + height/2.0 + cy
	print("(u6,v6)7 = "+str(u67)+":"+str(v67))
	landmark7_1 = R.dot(landmark7 - x7)
	u77 = - (f * landmark7_1[0] / landmark7_1[2]) + width/2.0 + cx
	v77 = f * landmark7_1[1] / landmark7_1[2] + height/2.0 + cy
	print("(u7,v7)7 = "+str(u77)+":"+str(v77))
	print(" ")
	
	# 8 座標
	x8 = np.array([0.71,-0.071,0.0])
	
	# 8 回転角
	ox = - pi / 2.0
	oy = 0.0
	oz = 0.07
	
	# 8 回転行列 W -> C
	rotXinv = Util.rotationMatrixX(-ox)
	rotYinv = Util.rotationMatrixY(-oy)
	rotZinv = Util.rotationMatrixZ(-oz)
	R = np.dot(rotXinv, np.dot(rotYinv, rotZinv))
	
	# 8 画像座標
	landmark1_1 = R.dot(landmark1 - x8)
	u18 = - (f * landmark1_1[0] / landmark1_1[2]) + width/2.0 + cx
	v18 = f * landmark1_1[1] / landmark1_1[2] + height/2.0 + cy
	print("(u1,v1)8 = "+str(u18)+":"+str(v18))
	landmark2_1 = R.dot(landmark2 - x8)
	u28 = - (f * landmark2_1[0] / landmark2_1[2]) + width/2.0 + cx
	v28 = f * landmark2_1[1] / landmark2_1[2] + height/2.0 + cy
	print("(u2,v2)8 = "+str(u28)+":"+str(v28))
	landmark3_1 = R.dot(landmark3 - x8)
	u38 = - (f * landmark3_1[0] / landmark3_1[2]) + width/2.0 + cx
	v38 = f * landmark3_1[1] / landmark3_1[2] + height/2.0 + cy
	print("(u3,v3)8 = "+str(u38)+":"+str(v38))
	landmark4_1 = R.dot(landmark4 - x8)
	u48 = - (f * landmark4_1[0] / landmark4_1[2]) + width/2.0 + cx
	v48 = f * landmark4_1[1] / landmark4_1[2] + height/2.0 + cy
	print("(u4,v4)8 = "+str(u48)+":"+str(v48))
	landmark5_1 = R.dot(landmark5 - x8)
	u58 = - (f * landmark5_1[0] / landmark5_1[2]) + width/2.0 + cx
	v58 = f * landmark5_1[1] / landmark5_1[2] + height/2.0 + cy
	print("(u5,v5)8 = "+str(u58)+":"+str(v58))
	landmark6_1 = R.dot(landmark6 - x8)
	u68 = - (f * landmark6_1[0] / landmark6_1[2]) + width/2.0 + cx
	v68 = f * landmark6_1[1] / landmark6_1[2] + height/2.0 + cy
	print("(u6,v6)8 = "+str(u68)+":"+str(v68))
	landmark7_1 = R.dot(landmark7 - x8)
	u78 = - (f * landmark7_1[0] / landmark7_1[2]) + width/2.0 + cx
	v78 = f * landmark7_1[1] / landmark7_1[2] + height/2.0 + cy
	print("(u7,v7)8 = "+str(u78)+":"+str(v78))
	print(" ")
	
	"""
	# ■ 座標
	x■ = np.array([0.★1,-0.0★1,0.0])
	
	# ■ 回転角
	ox = - pi / 2.0
	oy = 0.0
	oz = 0.0★
	
	# ■ 回転行列 W -> C
	rotXinv = Util.rotationMatrixX(-ox)
	rotYinv = Util.rotationMatrixY(-oy)
	rotZinv = Util.rotationMatrixZ(-oz)
	R = np.dot(rotXinv, np.dot(rotYinv, rotZinv))
	
	# ■ 画像座標
	landmark1_1 = R.dot(landmark1 - x■)
	u1■ = - (f * landmark1_1[0] / landmark1_1[2]) + width/2.0 + cx
	v1■ = f * landmark1_1[1] / landmark1_1[2] + height/2.0 + cy
	print("(u1,v1)■ = "+str(u1■)+":"+str(v1■))
	landmark2_1 = R.dot(landmark2 - x■)
	u2■ = - (f * landmark2_1[0] / landmark2_1[2]) + width/2.0 + cx
	v2■ = f * landmark2_1[1] / landmark2_1[2] + height/2.0 + cy
	print("(u2,v2)■ = "+str(u2■)+":"+str(v2■))
	landmark3_1 = R.dot(landmark3 - x■)
	u3■ = - (f * landmark3_1[0] / landmark3_1[2]) + width/2.0 + cx
	v3■ = f * landmark3_1[1] / landmark3_1[2] + height/2.0 + cy
	print("(u3,v3)■ = "+str(u3■)+":"+str(v3■))
	landmark4_1 = R.dot(landmark4 - x■)
	u4■ = - (f * landmark4_1[0] / landmark4_1[2]) + width/2.0 + cx
	v4■ = f * landmark4_1[1] / landmark4_1[2] + height/2.0 + cy
	print("(u4,v4)■ = "+str(u4■)+":"+str(v4■))
	landmark5_1 = R.dot(landmark5 - x■)
	u5■ = - (f * landmark5_1[0] / landmark5_1[2]) + width/2.0 + cx
	v5■ = f * landmark5_1[1] / landmark5_1[2] + height/2.0 + cy
	print("(u5,v5)■ = "+str(u5■)+":"+str(v5■))
	landmark6_1 = R.dot(landmark6 - x■)
	u6■ = - (f * landmark6_1[0] / landmark6_1[2]) + width/2.0 + cx
	v6■ = f * landmark6_1[1] / landmark6_1[2] + height/2.0 + cy
	print("(u6,v6)■ = "+str(u6■)+":"+str(v6■))
	landmark7_1 = R.dot(landmark7 - x■)
	u7■ = - (f * landmark7_1[0] / landmark7_1[2]) + width/2.0 + cx
	v7■ = f * landmark7_1[1] / landmark7_1[2] + height/2.0 + cy
	print("(u7,v7)■ = "+str(u7■)+":"+str(v7■))
	print(" ")
	"""
	
	# MQTT形式に変換
	print("SLAM/input/camera%120$1:1:"+str(u11)+":"+str(v11)+":"+str(u12)+":"+str(v12)+"&2:2:"+str(u21)+":"+str(v21)+":"+str(u22)+":"+str(v22)+"&3:3:"+str(u31)+":"+str(v31)+":"+str(u32)+":"+str(v32)+"&4:4:"+str(u41)+":"+str(v41)+":"+str(u42)+":"+str(v42)+"&5:5:"+str(u51)+":"+str(v51)+":"+str(u52)+":"+str(v52)+"&")
	print("SLAM/input/camera%220$1:1:"+str(u12)+":"+str(v12)+":"+str(u13)+":"+str(v12)+"&2:2:"+str(u22)+":"+str(v22)+":"+str(u23)+":"+str(v23)+"&3:3:"+str(u32)+":"+str(v32)+":"+str(u33)+":"+str(v33)+"&4:4:"+str(u42)+":"+str(v42)+":"+str(u43)+":"+str(v43)+"&5:5:"+str(u52)+":"+str(v52)+":"+str(u53)+":"+str(v53)+"&6:6:"+str(u62)+":"+str(v62)+":"+str(u63)+":"+str(v63)+"&7:7:"+str(u72)+":"+str(v72)+":"+str(u73)+":"+str(v73)+"&")
	print("SLAM/input/camera%320$1:1:"+str(u13)+":"+str(v13)+":"+str(u14)+":"+str(v14)+"&2:2:"+str(u23)+":"+str(v23)+":"+str(u24)+":"+str(v24)+"&3:3:"+str(u33)+":"+str(v33)+":"+str(u34)+":"+str(v34)+"&4:4:"+str(u43)+":"+str(v43)+":"+str(u44)+":"+str(v44)+"&5:5:"+str(u53)+":"+str(v53)+":"+str(u54)+":"+str(v54)+"&6:6:"+str(u63)+":"+str(v63)+":"+str(u64)+":"+str(v64)+"&7:7:"+str(u73)+":"+str(v73)+":"+str(u74)+":"+str(v74)+"&")	
	print("SLAM/input/camera%420$1:1:"+str(u14)+":"+str(v14)+":"+str(u15)+":"+str(v15)+"&2:2:"+str(u24)+":"+str(v24)+":"+str(u25)+":"+str(v25)+"&3:3:"+str(u34)+":"+str(v34)+":"+str(u35)+":"+str(v35)+"&4:4:"+str(u44)+":"+str(v44)+":"+str(u45)+":"+str(v45)+"&5:5:"+str(u54)+":"+str(v54)+":"+str(u55)+":"+str(v55)+"&6:6:"+str(u64)+":"+str(v64)+":"+str(u65)+":"+str(v65)+"&7:7:"+str(u74)+":"+str(v74)+":"+str(u75)+":"+str(v75)+"&")		
	print("SLAM/input/camera%520$1:1:"+str(u15)+":"+str(v15)+":"+str(u16)+":"+str(v16)+"&2:2:"+str(u25)+":"+str(v25)+":"+str(u26)+":"+str(v26)+"&3:3:"+str(u35)+":"+str(v35)+":"+str(u36)+":"+str(v36)+"&4:4:"+str(u45)+":"+str(v45)+":"+str(u46)+":"+str(v46)+"&5:5:"+str(u55)+":"+str(v55)+":"+str(u56)+":"+str(v56)+"&6:6:"+str(u65)+":"+str(v65)+":"+str(u66)+":"+str(v66)+"&7:7:"+str(u75)+":"+str(v75)+":"+str(u76)+":"+str(v76)+"&")		
	print("SLAM/input/camera%620$1:1:"+str(u16)+":"+str(v16)+":"+str(u17)+":"+str(v17)+"&2:2:"+str(u26)+":"+str(v26)+":"+str(u27)+":"+str(v27)+"&3:3:"+str(u36)+":"+str(v36)+":"+str(u37)+":"+str(v37)+"&4:4:"+str(u46)+":"+str(v46)+":"+str(u47)+":"+str(v47)+"&5:5:"+str(u56)+":"+str(v56)+":"+str(u57)+":"+str(v57)+"&6:6:"+str(u66)+":"+str(v66)+":"+str(u67)+":"+str(v67)+"&7:7:"+str(u76)+":"+str(v76)+":"+str(u77)+":"+str(v77)+"&")		
	print("SLAM/input/camera%720$1:1:"+str(u17)+":"+str(v17)+":"+str(u18)+":"+str(v18)+"&2:2:"+str(u27)+":"+str(v27)+":"+str(u28)+":"+str(v28)+"&3:3:"+str(u37)+":"+str(v37)+":"+str(u38)+":"+str(v38)+"&4:4:"+str(u47)+":"+str(v47)+":"+str(u48)+":"+str(v48)+"&5:5:"+str(u57)+":"+str(v57)+":"+str(u58)+":"+str(v58)+"&6:6:"+str(u67)+":"+str(v67)+":"+str(u68)+":"+str(v68)+"&7:7:"+str(u77)+":"+str(v77)+":"+str(u78)+":"+str(v78)+"&")		
		
	"""
	print("SLAM/input/camera%■20$1:1:"+str(u1★)+":"+str(v1★)+":"+str(u1■)+":"+str(v1■)+"&2:2:"+str(u2★)+":"+str(v2★)+":"+str(u2■)+":"+str(v2■)+"&3:3:"+str(u3★)+":"+str(v3★)+":"+str(u3■)+":"+str(v3■)+"&4:4:"+str(u4★)+":"+str(v4★)+":"+str(u4■)+":"+str(v4■)+"&5:5:"+str(u5★)+":"+str(v5★)+":"+str(u5■)+":"+str(v5■)+"&6:6:"+str(u6★)+":"+str(v6★)+":"+str(u6■)+":"+str(v6■)+"&7:7:"+str(u7★)+":"+str(v7★)+":"+str(u7■)+":"+str(v7■)+"&")	
	"""


if __name__ == '__main__':
	main()