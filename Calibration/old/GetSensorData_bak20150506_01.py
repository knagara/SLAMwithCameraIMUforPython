# -*- coding: utf-8 -*-

"""
GetSonsorData.py
Subscribe sensor data and save them as CSV file.
"""

import sys
import numpy as np
import paho.mqtt.client as mqtt


#This method is called when mqtt is connected.
def on_connect(client, userdata, flags, rc):
    print('Connected with result code '+str(rc))
    client.subscribe("SLAM/input/#")
				

#Append new data array to the array.
def appendData(sensor, msg, ary):
	global isFirst
	global time0
	
	data = msg.split(',')
	if(isFirst[sensor]):
		time0[sensor] = long(data[0])
	ary0 = np.array([long(data[0])-time0[sensor],float(data[1]),float(data[2]),float(data[3])])
	if(isFirst[sensor]):
		ary = ary0
		isFirst[sensor] = False
	else:
		ary = np.c_[ary, ary0]
	return ary


#This method is called when message is arrived.
def on_message(client, userdata, msg):
	global isFirst
	global time0
	global accel, accel_g, gyro, magnet, orientation

    #print(msg.topic + ' ' + str(msg.payload))

    #Append data to the array
	if(str(msg.topic) == "SLAM/input/acceleration"):
		accel = appendData("accel",str(msg.payload),accel)
	elif(str(msg.topic) == "SLAM/input/acceleration_with_gravity"):
		accel_g = appendData("accel_g",str(msg.payload),accel_g)
	elif(str(msg.topic) == "SLAM/input/gyro"):
		gyro = appendData("gyro",str(msg.payload),gyro)
	elif(str(msg.topic) == "SLAM/input/magnet"):
		magnet = appendData("magnet",str(msg.payload),magnet)
	elif(str(msg.topic) == "SLAM/input/orientation"):
		orientation = appendData("orientation",str(msg.payload),orientation)
	elif(str(msg.topic) == "SLAM/input/stop"):
		np.savetxt('./data/accel.csv', accel, delimiter=',')
		np.savetxt('./data/accel_g.csv', accel_g, delimiter=',')
		np.savetxt('./data/gyro.csv', gyro, delimiter=',')
		np.savetxt('./data/magnet.csv', magnet, delimiter=',')
		np.savetxt('./data/orientation.csv', orientation, delimiter=',')		
		sys.exit()


#Main method
if __name__ == '__main__':

	#global variables
	isFirst = {"accel":True, "accel_g":True, "gyro":True, "magnet":True, "orientation":True}
	time0 = {"accel":0, "accel_g":0, "gyro":0, "magnet":0, "orientation":0}
	accel = np.array([])
	accel_g = np.array([])
	gyro = np.array([])
	magnet = np.array([])
	orientation = np.array([])

	#Mqtt
	username = 'admin'
	password = 'password'
	host = 'vps01.t-spots.jp'
	port = 61713

	#Mqtt connect
	client = mqtt.Client(client_id="GetSensorData", clean_session=True, protocol=mqtt.MQTTv311)
	client.on_connect = on_connect
	client.on_message = on_message
	client.username_pw_set(username, password=password)
	client.connect(host, port=port, keepalive=60)
	client.loop_forever()
