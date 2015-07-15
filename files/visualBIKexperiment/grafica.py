#!/usr/bin/python

#
# Example boxplot code
#

from pylab import *

figure()


error_t = []
eee = []
number = 0
try:
	while True and number < 15:
                number += 1
		fname = "datosObtenidos_" + str(number).zfill(5) + ".txt"
		this = []
		for line in open(fname, 'r').readlines():
			err = line.split(' ')
			while '' in err: err.remove('')
			errT = err[7].split(':')
			print errT, err
			if int(err[18]) == 0:
                            this.append(float(errT[1]))
                        else:
                            print 'pasando de este'
		error_t.append(this)
		eee.append(number*5)
except IOError, ioerr:
	pass

fig, ax1 = plt.subplots(figsize=(10,6))
bp = plt.boxplot(error_t) #, notch=0, sym='+', vert=1, whis=1.5
fig.canvas.set_window_title('A Boxplot Example')
xtickNames = plt.setp(ax1, xticklabels=eee)
plt.setp(xtickNames)

#boxplot(error_t, 'gD')

			
# change outlier point symbols


show()

