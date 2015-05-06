# -*- coding: utf-8 -*-

"""
Main.py
"""

import sys
from math import *
import cv2 as cv
import numpy as np
import paho.mqtt.client as mqtt
from sensor import Sensor


#This method is called when mqtt is connected.
def on_connect(client, userdata, flags, rc):
    print('Connected with result code '+str(rc))
    client.subscribe("SLAM/input/#")


#This method is called when message is arrived.
def on_message(client, userdata, msg):
	global sensor, x

	data = str(msg.payload).split('&')
	#Append data to the array
	if(str(msg.topic) == "SLAM/input/all"):
		sensor.setData(data)
		ori = sensor.calcOrientation()
		sensor.calcGlobalAcceleration()
		x = sensor.localization()
		client.publish("SLAM/output/position",str(x[0])+"&"+str(x[1])+"&"+str(x[2]))
		client.publish("SLAM/output/orientation",str(ori[0])+"&"+str(ori[1])+"&"+str(ori[2]))
	elif(str(msg.topic) == "SLAM/input/stop"):
		print("stop")
		sensor.init()


#Main method
if __name__ == '__main__':

	#sensor.py
	sensor = Sensor()
	#position of device
	x = np.array([0.0,0.0,0.0])
	#orientation of device
	ori = np.array([0.0,0.0,0.0])

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
