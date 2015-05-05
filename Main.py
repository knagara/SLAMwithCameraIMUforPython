# -*- coding: utf-8 -*-

"""
Main.py
"""

import sys
from math import *
import cv2 as cv
import numpy as np
import paho.mqtt.client as mqtt


#initialize
def init():
	global isFirst, time
	global accel, accel_g, gyro, magnet, orientation
	global rot, x, x1, x2, v, v1, a, t, t1, t2
	
	isFirst = {"accel":True, "accel_g":True, "gyro":True, "magnet":True, "orientation":True}
	time = {"accel":0, "accel_g":0, "gyro":0, "magnet":0, "orientation":0}
	accel = np.array([])
	accel_g = np.array([])
	gyro = np.array([])
	magnet = np.array([])
	orientation = np.array([])
	rot = np.identity(3)
	x = np.array([0.0,0.0,0.0])
	x1 = np.array([0.0,0.0,0.0])
	x2 = np.array([0.0,0.0,0.0])
	v = np.array([0.0,0.0,0.0])
	v1 = np.array([0.0,0.0,0.0])
	a = np.array([0.0,0.0,0.0])
	t = 0
	t1 = 0
	t2 = 0


#This method is called when mqtt is connected.
def on_connect(client, userdata, flags, rc):
    print('Connected with result code '+str(rc))
    client.subscribe("SLAM/input/#")
				

#Set new data
def setData(sensor, data):
	global isFirst
	global time, t, t1 ,t2
	
	#set time
	if(isFirst[sensor]):
		time[sensor] = long(data[0])
	else:
		time[sensor] = long(data[0]) - time[sensor]
		
	#set time for localzation
	if(sensor=="orientation"):
		t2 = t1
		t1 = t
		t = float(long(data[0]) / 1000.0)
	
	#set data	
	if(sensor=="orientation"):
		ary = np.array([float(data[2]),float(data[3]),float(data[1])])
	else:
		ary = np.array([float(data[1]),float(data[2]),float(data[3])])
	return ary
	
	
#calc accel in global coordinates by using orientation
def calcGlobalAcceleration():
	global accel, accel_g, gyro, magnet, orientation
	global rot, a, a1
	
	a1 = a
	cv.Rodrigues(orientation,rot)
	a = np.dot(rot,accel)
	
	
#localization 
def localization():
	global client
	global t, t1 ,t2
	global x, x1, x2, v, v1, a, a1
	
	if(t1 == 0):
		return
		
	if(t2 == 0):
		v = (t - t1)*a1
		
	else:
		v1 = v
		v =  v1 + (t - t1)*a1
		x1 = x
		x = x1 + (t - t1)*v1 + 0.5*(t - t1)*(t - t1)*a1
		
		#print(x)
		client.publish("SLAM/output/location",str(x[0])+"&"+str(x[1])+"&"+str(x[2]))


#This method is called when message is arrived.
def on_message(client, userdata, msg):
	global time
	global accel, accel_g, gyro, magnet, orientation
	
	data = str(msg.payload).split(',')
	#Append data to the array
	if(str(msg.topic) == "SLAM/input/acceleration"):
		accel = setData("accel",data)
	elif(str(msg.topic) == "SLAM/input/acceleration_with_gravity"):
		accel_g = setData("accel_g",data)
	elif(str(msg.topic) == "SLAM/input/gyro"):
		gyro = setData("gyro",data)
	elif(str(msg.topic) == "SLAM/input/magnet"):
		magnet = setData("magnet",data)
	elif(str(msg.topic) == "SLAM/input/orientation"):
		orientation = setData("orientation",data)
		calcGlobalAcceleration()
		localization()
	elif(str(msg.topic) == "SLAM/input/stop"):
		print("stop")	
		init()


#Main method
if __name__ == '__main__':

	#global variables
	isFirst = {"accel":True, "accel_g":True, "gyro":True, "magnet":True, "orientation":True}
	time = {"accel":0, "accel_g":0, "gyro":0, "magnet":0, "orientation":0}
	accel = np.array([])
	accel_g = np.array([])
	gyro = np.array([])
	magnet = np.array([])
	orientation = np.array([])
	rot = np.identity(3)
	x = np.array([0.0,0.0,0.0])
	x1 = np.array([0.0,0.0,0.0])
	x2 = np.array([0.0,0.0,0.0])
	v = np.array([0.0,0.0,0.0])
	v1 = np.array([0.0,0.0,0.0])
	a = np.array([0.0,0.0,0.0])
	a1 = np.array([0.0,0.0,0.0])
	t = 0
	t1 = 0
	t2 = 0
	
	#Mqtt
	username = 'admin'
	password = 'password'
	host = 'vps01.t-spots.jp'
	port = 61713

	#Mqtt connect
	client = mqtt.Client(client_id="PyMain", clean_session=True, protocol=mqtt.MQTTv311)
	client.on_connect = on_connect
	client.on_message = on_message
	client.username_pw_set(username, password=password)
	client.connect(host, port=port, keepalive=60)
	client.loop_forever()
