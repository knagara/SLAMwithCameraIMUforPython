# -*- coding: utf-8 -*-

"""
GetImageData.py
Subscribe image data and save them as CSV file.
"""

import sys
import numpy as np
import paho.mqtt.client as mqtt


def init():
	global isFirst
	global time
	global output
	
	#global variables
	isFirst = True
	time = 0
	output =[]


#This method is called when mqtt is connected.
def on_connect(client, userdata, flags, rc):
    print('[GetImageData] Connected with result code '+str(rc))
    client.subscribe("SLAM/input/camera")
    client.subscribe("SLAM/input/stop")


#Append new data array to the array.
def appendData(time_,data):
	global isFirst
	global time
	global output
	
	timerow = np.array([long(time_),0,0,0,0,0])
	output.append(timerow)
	
	#if nomatch then nothing to do
	if(data[0] == "nomatch"):
		nomatchrow = np.array([-9999,0,0,0,0,0])
		output.append(nomatchrow)
		return

	for data_ in data:
		if(data_ != ''):
			d = data_.split(':')
			row = np.array([int(d[0]),float(d[2]),float(d[3]),int(d[1]),float(d[4]),float(d[5])])
			output.append(row)

	


#This method is called when message is arrived.
def on_message(client, userdata, msg):
	global isFirst
	global time
	global output

    #print(msg.topic + ' ' + str(msg.payload))

    #Append data to the array
	if(str(msg.topic) == "SLAM/input/camera"):
		data_ = str(msg.payload).split('$') # time$data&data&...
		time = data_[0]
		data = data_[1].split('&') # data&data&...
		appendData(time,data)
	elif(str(msg.topic) == "SLAM/input/stop"):
		np.savetxt('./input/image.csv', output, delimiter=',')
		print("[GetImageData] stop")
		init()


#Main method
if __name__ == '__main__':

	#global variables
	isFirst = True
	time = 0
	output = []

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
	client = mqtt.Client(client_id="GetImageData", clean_session=True, protocol=mqtt.MQTTv311)
	client.on_connect = on_connect
	client.on_message = on_message
	client.username_pw_set(username, password=password)
	client.connect(host, port=port, keepalive=60)
	client.loop_forever()
