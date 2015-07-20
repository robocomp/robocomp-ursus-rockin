#!/usr/bin/python

#
# Example boxplot code
#

from pylab import *

#figure()


error_R = []
eee = []
number = 0

#init_value = 0.0
#end_value  = (15.0*math.pi)/180.0#0.0
#step_value = end_value/10.0
init_value = 0.0
end_value  = 50
step_value = 5
stdDev_R  = init_value

try:
	while True and number < 15:
                number += 1
		fname = "datosObtenidos_" + str(number).zfill(5) + ".txt"
		this = []
		for line in open(fname, 'r').readlines():
			err = line.split(' ')
			while '' in err: err.remove('')
			errR = err[8].split(':')
			print errR, err
			#if int(err[18]) == 0:
			this.append(float(errR[1]))
			#else:
				#print 'pasando de este'
		error_R.append(this)
		eee.append(stdDev_R)
		stdDev_R += step_value
except IOError, ioerr:
	pass

fig, ax1 = plt.subplots(figsize=(10,6))
bp = plt.boxplot(error_R) #, notch=0, sym='+', vert=1, whis=1.5
fig.canvas.set_window_title('Grafica de Rotacion')
xtickNames = plt.setp(ax1, xticklabels=eee)
plt.setp(xtickNames)

#boxplot(error_t, 'gD')

			
# change outlier point symbols


show()

