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
from state import State


#This method is called when mqtt is connected.
def on_connect(client, userdata, flags, rc):
    print('Connected with result code '+str(rc))
    client.subscribe("SLAM/input/#")


#This method is called when message is arrived.
def on_message(client, userdata, msg):
	global sensor, x, a

	data = str(msg.payload).split('&')
	#Append data to the array
	if(str(msg.topic) == "SLAM/input/all"):
		sensor.setData(data)
		sensor.processData()
		x = state.getPosition()
		v = state.getVelocity()
		a = state.getAcceleration()
		ori = state.getOrientation()
		#print(x)
		client.publish("SLAM/output/accel",str(a[0])+"&"+str(a[1])+"&"+str(a[2]))
		client.publish("SLAM/output/velocity",str(v[0])+"&"+str(v[1])+"&"+str(v[2]))
		#client.publish("SLAM/output/position",str(x[0])+"&"+str(x[1])+"&"+str(x[2]))
		#client.publish("SLAM/output/orientation",str(ori[0])+"&"+str(ori[1])+"&"+str(ori[2]))
		client.publish("SLAM/output/all",str(x[0])+"&"+str(x[1])+"&"+str(x[2])+"&"+str(ori[0])+"&"+str(ori[1])+"&"+str(ori[2]))
	elif(str(msg.topic) == "SLAM/input/stop"):
		print("stop")
		client.publish("SLAM/output/stop","true")
		sensor.init()


#Main method
if __name__ == '__main__':

	#state.py
	state = State()
	#sensor.py
	sensor = Sensor(state)
	#position of device
	x = np.array([0.0,0.0,0.0])
	#orientation of device
	ori = np.array([0.0,0.0,0.0])
	#acceleration of device
	a = np.array([0.0,0.0,0.0])

	#Read server conf file
	f = open('../server.conf', 'r')
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
	client = mqtt.Client(client_id="PyMain", clean_session=True, protocol=mqtt.MQTTv311)
	client.on_connect = on_connect
	client.on_message = on_message
	client.username_pw_set(username, password=password)
	client.connect(host, port=port, keepalive=60)
	client.loop_forever()
