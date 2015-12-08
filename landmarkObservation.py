# -*- coding: utf-8 -*-
"""
landmarkObservation.py

author: Keita Nagara　永良慶太 (University of Tokyo) <nagara.keita()gmail.com>

"""

import theano
import theano.tensor as T
import numpy as np

class LandmarkObservation:
	
	def __init__(self):
		
		# ------------------- #
		# ---  variables  --- #
		# ------------------- #
		
		# focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p
		
		focus = T.dscalar('focus') # focus length od camera
		
		# variables of device state
		xt, yt, zt = T.dscalars('xt', 'yt', 'zt') # position
		ox, oy, oz = T.dscalars('ox', 'oy', 'oz') # orientation
		
		# variables of landmark state
		xi, yi, zi = T.dscalars('xi', 'yi', 'zi') # Device position at first observation
		theta, phi = T.dscalars('theta', 'phi') # theta, phi (Azimuth & elevation of the ray at first observation)
		p = T.dscalar('p') # d_inv (Inverse depth at first observation. 0.1 means depth is 10 meter.)
		
		# ------------------- #
		# --- observation --- #
		# ------------------- #
		# T.sin(ox)
		# T.cos(ox)
		# T.sin(oy)
		# T.cos(oy)
		# T.sin(oz)
		# T.cos(oz)
		# T.sin(theta)
		# T.cos(theta)
		# T.sin(phi)
		# T.cos(phi)
		
		# --- # h_ = [hx, hy, hz].T in the global coordinates --- #
		h_x = p*(xi - xt) + T.cos(phi)*T.sin(theta)
		h_y = p*(yi - yt) - T.sin(phi)
		h_z = p*(zi - zt) + T.cos(phi)*T.cos(theta)
		
		# ---- hx, hy, hz ---- #
		hx = (T.cos(oy)*T.cos(oz)) * h_x + (T.cos(oy)*T.sin(oz)) * h_y + (-T.sin(oy)) * h_z
		hy = (-T.cos(ox)*T.sin(oz)+T.sin(ox)*T.sin(oy)*T.cos(oz)) * h_x + (T.cos(ox)*T.cos(oz)+T.sin(ox)*T.sin(oy)*T.sin(oz)) * h_y + (T.sin(ox)*T.cos(oy)) * h_z
		hz = (T.sin(ox)*T.sin(oz)+T.cos(ox)*T.sin(oy)*T.cos(oz)) * h_x + (-T.sin(ox)*T.cos(oz) + T.cos(ox)*T.sin(oy)*T.sin(oz)) * h_y + (T.cos(ox)*T.cos(oy)) * h_z
		
		# --- h1, h2 --- #
		h1 = - (focus * hx / hz)
		h2 = focus * hy / hz
		
		self.fh1 = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=h1)
		self.fh2 = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=h2)
		
		# partial derivative
		dh1xt = T.grad(cost=h1, wrt=xt)
		dh1yt = T.grad(cost=h1, wrt=yt)
		dh1zt = T.grad(cost=h1, wrt=zt)
		dh1ox = T.grad(cost=h1, wrt=ox)
		dh1oy = T.grad(cost=h1, wrt=oy)
		dh1oz = T.grad(cost=h1, wrt=oz)
		dh1xi = T.grad(cost=h1, wrt=xi)
		dh1yi = T.grad(cost=h1, wrt=yi)
		dh1zi = T.grad(cost=h1, wrt=zi)
		dh1theta = T.grad(cost=h1, wrt=theta)
		dh1phi = T.grad(cost=h1, wrt=phi)
		dh1p = T.grad(cost=h1, wrt=p)
		
		dh2xt = T.grad(cost=h2, wrt=xt)
		dh2yt = T.grad(cost=h2, wrt=yt)
		dh2zt = T.grad(cost=h2, wrt=zt)
		dh2ox = T.grad(cost=h2, wrt=ox)
		dh2oy = T.grad(cost=h2, wrt=oy)
		dh2oz = T.grad(cost=h2, wrt=oz)
		dh2xi = T.grad(cost=h2, wrt=xi)
		dh2yi = T.grad(cost=h2, wrt=yi)
		dh2zi = T.grad(cost=h2, wrt=zi)
		dh2theta = T.grad(cost=h2, wrt=theta)
		dh2phi = T.grad(cost=h2, wrt=phi)
		dh2p = T.grad(cost=h2, wrt=p)
		
		# function of partial derivative
		self.fdh1xt = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh1xt)
		self.fdh1yt = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh1yt)
		self.fdh1zt = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh1zt)
		self.fdh1ox = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh1ox)
		self.fdh1oy = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh1oy)
		self.fdh1oz = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh1oz)
		self.fdh1xi = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh1xi)
		self.fdh1yi = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh1yi)
		self.fdh1zi = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh1zi)
		self.fdh1theta = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh1theta)
		self.fdh1phi = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh1phi)
		self.fdh1p = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh1p)
		
		self.fdh2xt = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh2xt)
		self.fdh2yt = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh2yt)
		self.fdh2zt = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh2zt)
		self.fdh2ox = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh2ox)
		self.fdh2oy = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh2oy)
		self.fdh2oz = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh2oz)
		self.fdh2xi = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh2xi)
		self.fdh2yi = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh2yi)
		self.fdh2zi = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh2zi)
		self.fdh2theta = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh2theta)
		self.fdh2phi = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh2phi)
		self.fdh2p = theano.function(inputs=[focus, xt, yt, zt, ox, oy, oz, xi, yi, zi, theta, phi, p], outputs=dh2p)
