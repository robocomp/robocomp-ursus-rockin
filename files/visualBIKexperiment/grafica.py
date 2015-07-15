#!/usr/bin/python

#
# Example boxplot code
#

from pylab import *

figure()


error_t = []
try:
	for number in xrange(1, 40):
		fname = "datosObtenidos_" + str(number).zfill(5) + ".txt"
		this = []
		for line in open(fname, 'r').readlines():
			err = line.split(' ')
			while '' in err: err.remove('')
			errT = err[7].split(':')
			print errT, err
			this.append(float(errT[1]))
		error_t.append(this)
except IOError, ioerr:
	pass
boxplot(error_t, 'gD')
			
# change outlier point symbols


show()

