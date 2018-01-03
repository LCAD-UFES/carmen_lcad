# -*- coding: utf-8 -*
import numpy as np
import caffe, json
"""
Compute the spatial dropout. In contrast to the standard dropout, complete feature maps are dropped out.
"""

__author__ = 'Timo Sämann'
__university__ = 'Aschaffenburg University of Applied Sciences'
__email__ = 'Timo.Saemann@gmx.de'
__data__ = '16th May, 2017'

class SpatialDropoutLayer(caffe.Layer):


    def setup(self, bottom, top):
        # check input pair
        if len(bottom) != 1:
            raise Exception("Dropout layer needs one blob as input; received bottom blobs = {}".format(len(bottom)))
	if len(top) != 1:
            raise Exception("Output of dropout layer is just one blob; received top blobs = {}".format(len(top)))

	#param = json.loads( self.param_str ) # use JSON to convert string to dict
	param = eval(self.param_str)
	self.phase = param['phase']
    	self.p = float(param['p'])
	if self.p <= 0 or self.p >= 1:
            raise Exception("p need to be between 0 and 1")
	self.scale_ = 1. / (1.-self.p)
	self.number_of_channels = bottom[0].channels
	self.number_of_num = bottom[0].num

	self.vec = np.ones(self.number_of_channels, dtype=np.int)


    def reshape(self, bottom, top):
        # Copy shape from bottom
        top[0].reshape(*bottom[0].data.shape)


    def forward(self, bottom, top):
	if self.phase == 'TRAIN':
		for i in xrange(self.number_of_num):
    			self.vec_w = np.random.binomial(self.vec, 1-self.p, size=None)
			for j in xrange(self.number_of_channels):
				top[0].data[i,j,:,:] = bottom[0].data[i,j,:,:] * self.vec_w[j] * self.scale_

	else:
        	top[0].data[...] = bottom[0].data[...] * self.scale_



    def backward(self, top, propagate_down, bottom):
	if propagate_down[0] == 1:
		if self.phase == 'TRAIN':
			for i in xrange(self.number_of_num):
				for j in xrange(self.number_of_channels):
					bottom[0].diff[i,j,:,:] = top[0].diff[i,j,:,:] * self.vec_w[j] * self.scale_

		else:
			bottom[0].diff[...] = top[0].diff[...] * self.scale_
	else:
		pass
