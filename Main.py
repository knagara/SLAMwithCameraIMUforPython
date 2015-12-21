# -*- coding: utf-8 -*-

"""
Main.py

author: Keita Nagara　永良慶太 (University of Tokyo) <nagara.keita()gmail.com>

The process starts from this program.
This program receives data from Android application via MQTT, and then send the data to Sensor class or Image class.
Data are:
  IMU sensor data     MQTT topic = SLAM/input/all    -> Sensor class
  Camera image data   MQTT topic = SLAM/input/camera -> Image class

Sensor class processes IMU sensor data, and send results to State class.
Image class processes Camera image data, and send results to State class.

State class estimates state (location of the device, etc.) in according to the model you selected below.

FYI:
MQTT is one of the lightweight messaging protocols.
If you want to run this program, you must prepare your own server and install MQTT broker, and create "server.conf" on the parent directory of this file. "server.conf" is the file like "ipaddress&port&username&password".

"""

import paho.mqtt.client as mqtt
from sensor import Sensor
from state import State
from image import Image
#from landmarkObservation import LandmarkObservation


# Main method
def main():
	global state, sensor, image

	# ============================== #
	#       Select model here!       #
	# ============================== #
	model = "RBPF"
	# ===== Model options (state vector type & estimation model) ===== #
	# - Coplanarity (IMU with Kalman Filter & Camera with Particle Filter. Observation model is coplanarity. State vector is device state only)
	# - RBPF (FastSLAM. IMU with Particle Filter & Camera with Extended Kalman Filter. Observation model is inverse depth. State vector are device and landmark state. Estimated by Rao-Blackwellized particle filter)
	# - IMUKF (IMU with Kalman Filter)
	# - IMUPF (IMU with Particle Filter, IMU data is observation)
	# - IMUPF2 (IMU with Particle Filter, IMU data is control)
	# ============================== #
	#       Select model here!       #
	# ============================== #

	print("=======================================")
	print("   SLAM with Camera and IMU")
	print("   "+model+" model is selected.")
	print("=======================================")

	state = State().getStateClass(model)
	sensor = Sensor(state)
	
	if(model == "Coplanarity" or model == "RBPF"):
		image = Image().getImageClass(model)
		image.setState(state)
	
	#print("Loading theano ... please wait")
	#observation = LandmarkObservation()
	#if(model == "RBPF"):
	#	state.setObservationModel(observation)

	#Get estimated state vector from state class and publish to the server (MQTT broker).
	def publish_state():
		global state, sensor, image
		
		x,v,a,o = state.getState() # Get estimated state vector
		
		if(len(sensor.accel) == 0):
			return

		a_temp = sensor.accel_g
		o_temp = sensor.orientation

		# Publish to the server (MQTT broker)
		client.publish("SLAM/output/all",str(x[0])+"&"+str(x[1])+"&"+str(x[2])+"&"+str(o[0])+"&"+str(o[1])+"&"+str(o[2]))
		client.publish("SLAM/output/accel",str(a[0])+"&"+str(a[1])+"&"+str(a[2]))
		client.publish("SLAM/output/velocity",str(v[0])+"&"+str(v[1])+"&"+str(v[2]))
		#client.publish("SLAM/output/temp",str(a[0])+"&"+str(a[1])+"&"+str(a[2])+"&"+str(a_temp[0])+"&"+str(a_temp[1])+"&"+str(a_temp[2]))
		#client.publish("SLAM/output/temp",str(o[0])+"&"+str(o[1])+"&"+str(o[2])+"&"+str(o_temp[0])+"&"+str(o_temp[1])+"&"+str(o_temp[2]))


	#This method is called when mqtt is connected.
	def on_connect(client, userdata, flags, rc):
	    print('[Main.py] Connected with result code '+str(rc))
	    client.subscribe("SLAM/input/#")


	#This method is called when message is arrived.
	def on_message(client, userdata, msg):
		global state, sensor, image


		#Process data in senser.py or image.py
		if(str(msg.topic) == "SLAM/input/camera"): #image.py
			if(model == "Coplanarity" or model == "RBPF"):
				#print("+"),
				data_ = str(msg.payload).split('$') # time$data&data&...
				time = data_[0]
				data = data_[1].split('&') # data&data&...
				image.processData(time,data) #Process data
				publish_state()
				#print "|",

		elif(str(msg.topic) == "SLAM/input/all"): #sensor.py
			#print "*",
			data = str(msg.payload).split('&') # data&data&...
			sensor.processData(data) #Process data
			publish_state()
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
	client = mqtt.Client(client_id="SLAMpython", clean_session=True, protocol=mqtt.MQTTv311)
	client.on_connect = on_connect
	client.on_message = on_message
	client.username_pw_set(username, password=password)
	client.connect(host, port=port, keepalive=60)
	client.loop_forever()


#Main method
if __name__ == '__main__':
	main()
