# -*- coding: utf-8 -*-

import csv

filename = 'passage02'
 
csvfile = './output/'+filename+'.csv'
f = open(csvfile, "r")
reader = csv.reader(f)
header = next(reader)

i = 0
x = []
y = []
z = []

for row in reader:
	if(i==0):
		x = row
	elif(i==1):
		y = row
	elif(i==2):
		z = row
	else:
		pass
	i+=1
	
f.close()

f = open('./output/'+filename+'.txt', 'w')

for i in range(len(x)):
	f.write('SLAM/output/all%'+x[i].strip()+'&'+y[i].strip()+'&'+z[i].strip()+'\n')
	
f.write('SLAM/input/stop%true')
f.close()