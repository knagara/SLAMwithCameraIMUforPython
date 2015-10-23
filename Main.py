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
If you want to run this program, you must prepare your own server and install MQTT broker, and make "server.conf" on the parent directory of this file. "server.conf" is the file like "hostIPaddress&portNumber&username&password".
"""

import paho.mqtt.client as mqtt
from sensor import Sensor
from state import State
from image import Image


# Main method
def main():
	global state, sensor, image
	
	# Select model (state type & estimation model)
	# - IMUKF (IMU with Kalman Filter)
	# - IMUPF (IMU with Particle Filter, IMU data is observation)
	# - IMUPF2 (IMU with Particle Filter, IMU data is control)
	state = State().getStateClass("IMUPF2")
	#sensor.py
	sensor = Sensor(state)
	#image.py
	image = Image(state)
	

	#This method is called when mqtt is connected.
	def on_connect(client, userdata, flags, rc):
	    print('[Main.py] Connected with result code '+str(rc))
	    client.subscribe("SLAM/input/#")
	
	
	#This method is called when message is arrived.
	def on_message(client, userdata, msg):
		global state, sensor, image
		
		data = str(msg.payload).split('&') #split message by &
		
		#Process data in senser.py or image.py
		if(str(msg.topic) == "SLAM/input/camera"): #image.py
			#print("+"),
			image.processData(data) #Process data
			#print "|",
			
		elif(str(msg.topic) == "SLAM/input/all"): #sensor.py
			#print "*",
			sensor.processData(data) #Process data
			
			x,v,a,o = state.getState() #Get estimated state vector
			a_temp = sensor.accel_g
			
			#Publish estimated state vector to MQTT broker
			client.publish("SLAM/output/all",str(x[0])+"&"+str(x[1])+"&"+str(x[2])+"&"+str(o[0])+"&"+str(o[1])+"&"+str(o[2]))
			client.publish("SLAM/output/accel",str(a[0])+"&"+str(a[1])+"&"+str(a[2]))
			client.publish("SLAM/output/velocity",str(v[0])+"&"+str(v[1])+"&"+str(v[2]))
			client.publish("SLAM/output/temp",str(a[0])+"&"+str(a[1])+"&"+str(a[2])+"&"+str(a_temp[0])+"&"+str(a_temp[1])+"&"+str(a_temp[2]))
			#print ",",
	
		elif(str(msg.topic) == "SLAM/input/stop"): #Stop
			print("[Main.py] stop")
			client.publish("SLAM/output/stop","true")
			state.init()
			sensor.init()


	#Read server conf file
	f = open('../server.conf', 'r')
	for line in f:
		serverconf = line
	f.close()

	#Mqtt connection args
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


#Main method
if __name__ == '__main__':
	main()
