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
	global accel, gravity, gyro, magnet
	global rot, x, x1, x2, v, v1, a, t, t1, t2

	accel = np.array([])
	gravity = np.array([])
	magnet = np.array([])
	gyro = np.array([])
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
def setData(data):
	global accel, gravity, gyro, magnet
	global t, t1 ,t2

	#set time for localzation
	t2 = t1
	t1 = t
	t = float(long(data[0]) / 1000.0)

	#set data
	accel = np.array([-float(data[1]),-float(data[2]),-float(data[3])])
	gravity = np.array([-float(data[4]),-float(data[5]),-float(data[6])])
	magnet = np.array([float(data[7]),float(data[8]),float(data[9])])
	gyro = np.array([float(data[10]),float(data[11]),float(data[12])])


#calc orientation by using gravity and magnet
def calcOrientation():
	global gravity, magnet, orientation

	#roll
	orientation[0] = atan2(gravity[1],gravity[2])
	#pitch
	orientation[1] = atan2(-gravity[0],sqrt(pow(gravity[1],2)+pow(gravity[2],2)))
	#yaw
	rotX = np.identity(3)
	rotY = np.identity(3)
	cv.Rodrigues(np.array((orientation[0],0.0,0.0)),rotX)
	cv.Rodrigues(np.array((0.0,orientation[1],0.0)),rotY)
	rotXY = np.dot(rotY,rotX)
	magnet_fixed = np.dot(rotXY,magnet)
	orientation[2] = atan2(-magnet_fixed[1],magnet_fixed[0])

	print(str(degrees(orientation[0]))+" "+str(degrees(orientation[1]))+" "+str(degrees(orientation[2])))
	#print(str(degrees(orientation[2])))
	#print(rotXY)


#calc accel in global coordinates by using orientation
def calcGlobalAcceleration():
	global accel, gravity, gyro, magnet, orientation
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
	data = str(msg.payload).split(',')
	#Append data to the array
	if(str(msg.topic) == "SLAM/input/all"):
		setData(data)
		calcOrientation()
		#calcGlobalAcceleration()
		#localization()
	elif(str(msg.topic) == "SLAM/input/stop"):
		print("stop")
		init()


#Main method
if __name__ == '__main__':

	#global variables
	accel = np.array([])
	gravity = np.array([])
	magnet = np.array([])
	gyro = np.array([])
	orientation = np.array([0.0,0.0,0.0])
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
