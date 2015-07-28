# -*- coding: utf-8 -*-

"""
Main.py

author: Keita Nagara (University of Tokyo)


All process start from this program.
This program receives sensor data from Android application via MQTT message,
and then it calls methods of "sensor.py".
"sensor.py" processes sensor data, and results are stored in "state.py".

FYI:
MQTT is one of the lightweight messaging protocols.
If you want to run this program, you must prepare your own server and install MQTT broker, and make "server.conf" on the parent directory. "server.conf" is the file like "hostIPaddress&portNumber&username&password".
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
    print('[Main.py] Connected with result code '+str(rc))
    client.subscribe("SLAM/input/#")


#This method is called when message is arrived.
def on_message(client, userdata, msg):
	global sensor, x, a
	data = str(msg.payload).split('&')
	#Append data to the array
	if(str(msg.topic) == "SLAM/input/all"):
		sensor.processData(data)
		x = state.getPosition()
		v = state.getVelocity()
		a = state.getAcceleration()
		#a = sensor.accel #
		ori = state.getOrientation()
		#temp
		#temp1 = sensor.orientation #
		#temp2 = sensor.orientation_gyro #
		#temp3 = sensor.orientation_g #
		temp1 = sensor.centrifugal
		temp2 = sensor.tangential
		#temp3 = sensor.accel
		temp3 = sensor.r
		#temp1 = sensor.gyro
		#temp2 = sensor.angularAccel

		print "*",

		#print '%0.2f %0.2f %0.2f' % (temp[0],temp[1],temp[2])

		client.publish("SLAM/output/accel",str(a[0])+"&"+str(a[1])+"&"+str(a[2]))
		#client.publish("SLAM/output/accel",str(a[0])+"&"+str(a[1])+"&"+str(a[2])+"&"+str(a_[0])+"&"+str(a_[1])+"&"+str(a_[2]))

		client.publish("SLAM/output/velocity",str(v[0])+"&"+str(v[1])+"&"+str(v[2]))

		#client.publish("SLAM/output/temp",str(temp[0])+"&"+str(temp[1])+"&"+str(temp[2]))
		#client.publish("SLAM/output/temp",str(temp1[0])+"&"+str(temp1[1])+"&"+str(temp1[2])+"&"+str(temp2[0])+"&"+str(temp2[1])+"&"+str(temp2[2]))
		client.publish("SLAM/output/temp",str(temp1[0])+"&"+str(temp1[1])+"&"+str(temp1[2])+"&"+str(temp2[0])+"&"+str(temp2[1])+"&"+str(temp2[2])+"&"+str(temp3[0])+"&"+str(temp3[1])+"&"+str(temp3[2]))


		client.publish("SLAM/output/all",str(x[0])+"&"+str(x[1])+"&"+str(x[2])+"&"+str(ori[0])+"&"+str(ori[1])+"&"+str(ori[2]))
		#client.publish("SLAM/output/all",str(x[0])+"&"+str(x[1])+"&"+str(x[2])+"&"+str(ori[0])+"&"+str(ori[1])+"&"+str(ori[2])+"&"+str(temp1[0])+"&"+str(temp1[1])+"&"+str(temp1[2]))

	elif(str(msg.topic) == "SLAM/input/stop"):
		print("[Main.py] stop")
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
