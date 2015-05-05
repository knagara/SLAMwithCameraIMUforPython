# -*- coding: utf-8 -*-

import sys
import numpy as np
import paho.mqtt.client as mqtt

#This method is called when mqtt is connected.
def on_connect(client, userdata, flags, rc):
    print('Connected with result code '+str(rc))
    client.subscribe("SLAM/input/#")
				
#This method is called when message is arrived.
def on_message(client, userdata, msg):
	pass