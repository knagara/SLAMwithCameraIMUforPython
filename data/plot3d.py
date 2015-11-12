# -*- coding: utf-8 -*-

from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
from scipy import genfromtxt


# ファイル読み込み
d = genfromtxt("./plot3d/particles_before.csv", delimiter=",")

# グラフ作成
fig = pyplot.figure()
ax = Axes3D(fig)

# 軸ラベルの設定
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")

# 表示範囲の設定
ax.set_xlim(-0.2, 0.2)
ax.set_ylim(-0.2, 0.2)
ax.set_zlim(-0.2, 0.2)

# 抽出条件設定
#d1 = d[d[:,0] >= 7]
#d2 = d[(d[:,0] < 7) & ((d[:,1] > 3) & (d[:,1] <= 3.5))]
#d3 = d[(d[:,0] < 7) & ((d[:,1] <= 3) | (d[:,1] > 3.5))]


# グラフ描画
ax.plot(d[:,0], d[:,1], d[:,2], "o", color="#ff0000", ms=4, mew=0.5)
#ax.plot(d1[:,0], d1[:,1], d1[:,2], "o", color="#cccccc", ms=4, mew=0.5)
#ax.plot(d2[:,0], d2[:,1], d2[:,2], "o", color="#00cccc", ms=4, mew=0.5)
#ax.plot(d3[:,0], d3[:,1], d3[:,2], "o", color="#ff0000", ms=4, mew=0.5)
pyplot.show()