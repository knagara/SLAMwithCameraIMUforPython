# -*- coding: utf-8 -*-

"""
GetOutputDataTemp.py
Subscribe output and save them as CSV file.
"""

import sys
import numpy as np
import paho.mqtt.client as mqtt

def init():
	global isFirst, time
	global output

	isFirst = True
	time = 0
	output = np.array([])


#This method is called when mqtt is connected.
def on_connect(client, userdata, flags, rc):
    print('[GetOutputDataTemp] Connected with result code '+str(rc))
    client.subscribe("SLAM/output/#")


#Append new data array to the array.
def appendData(data):
	global isFirst
	global time
	global output

	time = time + 1
	#output0 = np.array([time,float(data[0]),float(data[1]),float(data[2])])
	output0 = np.array([time,float(data[0]),float(data[1]),float(data[2]),float(data[3]),float(data[4]),float(data[5])])
	#output0 = np.array([time,float(data[0]),float(data[1]),float(data[2]),float(data[3]),float(data[4]),float(data[5]),float(data[6]),float(data[7]),float(data[8])])

	if(isFirst):
		output = output0
		isFirst = False
	else:
		output = np.c_[output, output0]


#This method is called when message is arrived.
def on_message(client, userdata, msg):
	global isFirst
	global time
	global output

    #print(msg.topic + ' ' + str(msg.payload))

	data = str(msg.payload).split('&')
    #Append data to the array
	if(str(msg.topic) == "SLAM/output/temp"):
		appendData(data)
	elif(str(msg.topic) == "SLAM/output/stop"):
		np.savetxt('./output/temp.csv', output, delimiter=',')
		print("[GetOutputDataTemp] stop")
		init()


#Main method
if __name__ == '__main__':

	#global variables
	isFirst = True
	time = 0
	output = np.array([])

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
	client = mqtt.Client(client_id="GetOutputDataTemp", clean_session=True, protocol=mqtt.MQTTv311)
	client.on_connect = on_connect
	client.on_message = on_message
	client.username_pw_set(username, password=password)
	client.connect(host, port=port, keepalive=60)
	client.loop_forever()
