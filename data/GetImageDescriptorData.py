# -*- coding: utf-8 -*-

"""
GetImageDescriptorData.py
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
    print('[GetImageDescriptor] Connected with result code '+str(rc))
    client.subscribe("SLAM/input/camera")
    client.subscribe("SLAM/input/stop")


#Append new data array to the array.
def appendData(time_,data):
	global isFirst
	global time
	global output
	
	timerow = np.array([long(time_),0,0,0,0,0,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0])
	output.append(timerow)
	
	#if nomatch then nothing to do
	if(data[0] == "nomatch"):
		nomatchrow = np.array([-9999,0,0,0,0,0,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0])
		output.append(nomatchrow)
		return

	for data_ in data:
		if(data_ != ''):
			d = data_.split(':')
			row = np.array([int(d[0]),float(d[2]),float(d[3]),int(d[1]),float(d[4]),float(d[5]),float(d[6]), float(d[7]), float(d[8]), float(d[9]), float(d[10]), float(d[11]), float(d[12]), float(d[13]), float(d[14]), float(d[15]), float(d[16]), float(d[17]), float(d[18]), float(d[19]), float(d[20]), float(d[21]), float(d[22]), float(d[23]), float(d[24]), float(d[25]), float(d[26]), float(d[27]), float(d[28]), float(d[29]), float(d[30]), float(d[31]), float(d[32]), float(d[33]), float(d[34]), float(d[35]), float(d[36]), float(d[37]), float(d[38]), float(d[39]), float(d[40]), float(d[41]), float(d[42]), float(d[43]), float(d[44]), float(d[45]), float(d[46]), float(d[47]), float(d[48]), float(d[49]), float(d[50]), float(d[51]), float(d[52]), float(d[53]), float(d[54]), float(d[55]), float(d[56]), float(d[57]), float(d[58]), float(d[59]), float(d[60]), float(d[61]), float(d[62]), float(d[63]), float(d[64]), float(d[65]), float(d[66]), float(d[67]), float(d[68]), float(d[69])])
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
		np.savetxt('./input/image_descriptor.csv', output, delimiter=',')
		print("[GetImageDescriptor] stop")
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
	client = mqtt.Client(client_id="GetImageDescriptor", clean_session=True, protocol=mqtt.MQTTv311)
	client.on_connect = on_connect
	client.on_message = on_message
	client.username_pw_set(username, password=password)
	client.connect(host, port=port, keepalive=60)
	client.loop_forever()
