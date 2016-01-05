# -*- coding: utf-8 -*-

"""
GetImageDataSingle.py
Subscribe image data and plot on single image.
"""

import sys
import cv2
import numpy as np
import paho.mqtt.client as mqtt


def init():
	global isFirst
	global time
	global output
	global count

	#global variables
	isFirst = True
	time = 0
	output =[]
	count = 0


#This method is called when mqtt is connected.
def on_connect(client, userdata, flags, rc):
    print('[GetImagePlotSingle] Connected with result code '+str(rc))
    client.subscribe("SLAM/input/camera")
    client.subscribe("SLAM/input/stop")


#Append new data array to the array.
def appendData(time_,data):
	global isFirst
	global time
	global output
	global count

	count += 1

	timerow = np.array([-10000,0,0,0,0,0])
	output.append(timerow)

	#if nomatch then nothing to do
	if(data[0] == "nomatch"):
		nomatchrow = np.array([-9999,0,0,0,0,0])
		output.append(nomatchrow)
		return

	for data_ in data:
		if(data_ != ''):
			d = data_.split(':')
			row = np.array([int(d[0]),int(d[1]),float(d[2]),float(d[3]),float(d[4]),float(d[5])])
			output.append(row)


def plotImage():
	global output, count

	rows = 1280
	cols = 720
	#rows = 1920
	#cols = 1280
	img = np.zeros((rows, cols, 3), np.uint8)

	white = (255, 255, 255)
	green = (0, 255, 0)
	blue = (255, 200, 0)
	yellow = (0, 255, 255)

	font = cv2.FONT_HERSHEY_PLAIN
	font_size = 0.7

	for d in output:
		if(d[0] == -10000):
			pass
		elif(d[0] == -9999):
			#cv2.putText(img, "no match", (100 + i*1080,800), font, 10, yellow)
			pass
		else:
			color = blue

			x1 = int(d[2])
			y1 = int(d[3])
			x2 = int(d[4])
			y2 = int(d[5])
			#cv2.line(img, (x1,y1), (x2,y2), color, 1)
			cv2.line(img, (x1,y1), (x1,y1), yellow, 10)
			cv2.line(img, (x2,y2), (x2,y2), yellow, 10)
			#cv2.putText(img, str(int(d[0])), (x1+2,y1), font, font_size, color)
			#cv2.putText(img, str(int(d[1])), (x2-15,y2), font, font_size, color)

	#hight = img.shape[0]
	#width = img.shape[1]
	#img_half = cv2.resize(img,(hight/2,width/2))
	#cv2.imwrite('./input/plot.jpg', img_half)

	cv2.imwrite('./input/plot_single.jpg', img)




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
		#np.savetxt('./input/image.csv', output, delimiter=',')
		plotImage()
		print("[GetImagePlotSingle] stop")
		init()


#Main method
if __name__ == '__main__':

	#global variables
	isFirst = True
	time = 0
	output = []
	count = 0

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
	client = mqtt.Client(client_id="GetImagePlotSingle", clean_session=True, protocol=mqtt.MQTTv311)
	client.on_connect = on_connect
	client.on_message = on_message
	client.username_pw_set(username, password=password)
	client.connect(host, port=port, keepalive=60)
	client.loop_forever()
