# -*- coding: utf-8 -*-
import sys
import numpy as np
import paho.mqtt.client as mqtt


def on_connect(client, userdata, flags, rc):
    print('Connected with result code '+str(rc))
    client.subscribe("SLAM/input/#")


def on_message(client, userdata, msg):
	global isFirst
	global time0, time
	global accel_x, accel_y, accel_z
	global accel_g_x, accel_g_y, accel_g_z
	global gyro_x, gyro_y, gyro_z
	global magnet_x, magnet_y, magnet_z

    #print(msg.topic + ' ' + str(msg.payload))

    #Append data to array
	if(str(msg.topic) == "SLAM/input/time"):
		if(isFirst):
			time0 = long(str(msg.payload))
		time = np.append(time, long(str(msg.payload)) - time0)
	elif(str(msg.topic) == "SLAM/input/acceleration/x"):
		accel_x = np.append(accel_x, float(str(msg.payload)))
	elif(str(msg.topic) == "SLAM/input/acceleration/y"):
		accel_y = np.append(accel_y, float(str(msg.payload)))
	elif(str(msg.topic) == "SLAM/input/acceleration/z"):
		accel_z = np.append(accel_z, float(str(msg.payload)))
	elif(str(msg.topic) == "SLAM/input/acceleration_with_gravity/x"):
		accel_g_x = np.append(accel_g_x, float(str(msg.payload)))
	elif(str(msg.topic) == "SLAM/input/acceleration_with_gravity/y"):
		accel_g_y = np.append(accel_g_y, float(str(msg.payload)))
	elif(str(msg.topic) == "SLAM/input/acceleration_with_gravity/z"):
		accel_g_z = np.append(accel_g_z, float(str(msg.payload)))
	elif(str(msg.topic) == "SLAM/input/gyro/x"):
		gyro_x = np.append(gyro_x, float(str(msg.payload)))
	elif(str(msg.topic) == "SLAM/input/gyro/y"):
		gyro_y = np.append(gyro_y, float(str(msg.payload)))
	elif(str(msg.topic) == "SLAM/input/gyro/z"):
		gyro_z = np.append(gyro_z, float(str(msg.payload)))
	elif(str(msg.topic) == "SLAM/input/magnet/x"):
		magnet_x = np.append(magnet_x, float(str(msg.payload)))
	elif(str(msg.topic) == "SLAM/input/magnet/y"):
		magnet_y = np.append(magnet_y, float(str(msg.payload)))
	elif(str(msg.topic) == "SLAM/input/magnet/z"):
		magnet_z = np.append(magnet_z, float(str(msg.payload)))
	elif(str(msg.topic) == "SLAM/input/stop"):
		"""
		np.savetxt('./data/time.txt', time, delimiter=',')
		np.savetxt('./data/accel_x.txt', accel_x, delimiter=',')
		np.savetxt('./data/accel_y.txt', accel_y, delimiter=',')
		np.savetxt('./data/accel_z.txt', accel_z, delimiter=',')
		np.savetxt('./data/accel_g_x.txt', accel_g_x, delimiter=',')
		np.savetxt('./data/accel_g_y.txt', accel_g_y, delimiter=',')
		np.savetxt('./data/accel_g_z.txt', accel_g_z, delimiter=',')
		np.savetxt('./data/gyro_x.txt', gyro_x, delimiter=',')
		np.savetxt('./data/gyro_y.txt', gyro_y, delimiter=',')
		np.savetxt('./data/gyro_z.txt', gyro_z, delimiter=',')
		np.savetxt('./data/magnet_x.txt', magnet_x, delimiter=',')
		np.savetxt('./data/magnet_y.txt', magnet_y, delimiter=',')
		np.savetxt('./data/magnet_z.txt', magnet_z, delimiter=',')
		"""
		accel_x_t = np.c_[time,accel_x]
		accel_y_t = np.c_[time,accel_y]
		accel_z_t = np.c_[time,accel_z]
		accel_g_x_t = np.c_[time,accel_g_x]
		accel_g_y_t = np.c_[time,accel_g_y]
		accel_g_z_t = np.c_[time,accel_g_z]
		gyro_x_t = np.c_[time,gyro_x]
		gyro_y_t = np.c_[time,gyro_y]
		gyro_z_t = np.c_[time,gyro_z]
		magnet_x_t = np.c_[time,magnet_x]
		magnet_y_t = np.c_[time,magnet_y]
		magnet_z_t = np.c_[time,magnet_z]
		np.savetxt('accel_x.txt', accel_x_t, delimiter=',')
		np.savetxt('accel_y.txt', accel_y_t, delimiter=',')
		np.savetxt('accel_z.txt', accel_z_t, delimiter=',')
		np.savetxt('accel_g_x.txt', accel_g_x_t, delimiter=',')
		np.savetxt('accel_g_y.txt', accel_g_y_t, delimiter=',')
		np.savetxt('accel_g_z.txt', accel_g_z_t, delimiter=',')
		np.savetxt('gyro_x.txt', gyro_x_t, delimiter=',')
		np.savetxt('gyro_y.txt', gyro_y_t, delimiter=',')
		np.savetxt('gyro_z.txt', gyro_z_t, delimiter=',')
		np.savetxt('magnet_x.txt', magnet_x_t, delimiter=',')
		np.savetxt('magnet_y.txt', magnet_y_t, delimiter=',')
		np.savetxt('magnet_z.txt', magnet_z_t, delimiter=',')
		
		sys.exit()

	if(isFirst):
		isFirst = False


if __name__ == '__main__':

	#global variables
	isFirst = True
	time0 = 0
	time = np.array([])
	accel_x = np.array([])
	accel_y = np.array([])
	accel_z = np.array([])
	accel_g_x = np.array([])
	accel_g_y = np.array([])
	accel_g_z = np.array([])
	gyro_x = np.array([])
	gyro_y = np.array([])
	gyro_z = np.array([])
	magnet_x = np.array([])
	magnet_y = np.array([])
	magnet_z = np.array([])

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
