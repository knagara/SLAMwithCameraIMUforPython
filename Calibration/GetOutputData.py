# -*- coding: utf-8 -*-

"""
GetOutputData.py
Subscribe output and save them as CSV file.
"""

import sys
import numpy as np
import paho.mqtt.client as mqtt

def init():	
	global isFirst, time, isFirstV, timeV, isFirstAll, timeAll
	global accel, velocity, gravity, gyro, magnet, ori, position, orientation
	
	isFirst = True
	time = 0
	isFirstV = True
	timeV = 0
	isFirstAll = True
	timeAll = 0
	accel = np.array([])
	velocity = np.array([])
	gravity = np.array([])
	gyro = np.array([])
	magnet = np.array([])
	ori = np.array([])
	position = np.array([])
	orientation = np.array([])


#This method is called when mqtt is connected.
def on_connect(client, userdata, flags, rc):
    print('Connected with result code '+str(rc))
    client.subscribe("SLAM/output/#")


#Append new data array to the array.
def appendData(data):
	global isFirst
	global time
	global accel, velocity, gravity, gyro, magnet, ori, position, orientation

	time = time + 1
	accel0 = np.array([time,float(data[0]),float(data[1]),float(data[2])])

	if(isFirst):
		accel = accel0
		isFirst = False
	else:
		accel = np.c_[accel, accel0]


#Append new data array to the array.
def appendDataVelocity(data):
	global isFirstV
	global timeV
	global accel, velocity, gravity, gyro, magnet, ori, position, orientation

	timeV = timeV + 1
	velocity0 = np.array([timeV,float(data[0]),float(data[1]),float(data[2])])

	if(isFirstV):
		velocity = velocity0
		isFirstV = False
	else:
		velocity = np.c_[velocity, velocity0]


#Append new data array to the array.
def appendDataAll(data):
	global isFirstAll
	global timeAll
	global accel, velocity, gravity, gyro, magnet, ori, position, orientation

	timeAll = timeAll + 1
	position0 = np.array([timeAll,float(data[0]),float(data[1]),float(data[2])])
	orientation0 = np.array([timeAll,float(data[3]),float(data[4]),float(data[5])])

	if(isFirstAll):
		position = position0
		orientation = orientation0
		isFirstAll = False
	else:
		position = np.c_[position, position0]
		orientation = np.c_[orientation, orientation0]

	#print(str(orientation0[1])+" "+str(orientation0[2])+" "+str(orientation0[3]))


#This method is called when message is arrived.
def on_message(client, userdata, msg):
	global isFirst
	global time
	global accel, velocity, gravity, gyro, magnet, ori, position, orientation

    #print(msg.topic + ' ' + str(msg.payload))

	data = str(msg.payload).split('&')
    #Append data to the array
	if(str(msg.topic) == "SLAM/output/accel"):
		appendData(data)
	if(str(msg.topic) == "SLAM/output/velocity"):
		appendDataVelocity(data)
	elif(str(msg.topic) == "SLAM/output/all"):
		appendDataAll(data)
	elif(str(msg.topic) == "SLAM/output/stop"):
		np.savetxt('./output/accel.csv', accel, delimiter=',')
		np.savetxt('./output/velocity.csv', velocity, delimiter=',')
		np.savetxt('./output/position.csv', position, delimiter=',')
		np.savetxt('./output/orientation.csv', orientation, delimiter=',')
		print("stop")
		init()


#Main method
if __name__ == '__main__':

	#global variables
	isFirst = True
	time = 0
	isFirstV = True
	timeV = 0
	isFirstAll = True
	timeAll = 0
	accel = np.array([])
	velocity = np.array([])
	gravity = np.array([])
	gyro = np.array([])
	magnet = np.array([])
	ori = np.array([])
	position = np.array([])
	orientation = np.array([])

	#Read server conf file
	f = open('../../server.conf', 'r')
	for line in f:
		serverconf = line
	f.close()

	#Mqtt
	conf = serverconf.split('&')
	host = conf[0]
	port = int(conf[1])
	username = conf[2]
	password = conf[3]

	#Mqtt connect
	client = mqtt.Client(client_id="GetSensorData", clean_session=True, protocol=mqtt.MQTTv311)
	client.on_connect = on_connect
	client.on_message = on_message
	client.username_pw_set(username, password=password)
	client.connect(host, port=port, keepalive=60)
	client.loop_forever()
