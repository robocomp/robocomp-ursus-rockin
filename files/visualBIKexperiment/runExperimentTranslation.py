

from generateErrors import *


import numpy as np

init_value = 0.0
end_value = 50.0
step_value = 0.5

for stdDev_T in np.arange(init_value, end_value+0.0001, step_value):
	print 'Running experiment with error in translation: stdDev_T='+str(stdDev_T)



	generateErrorsXML("ursus.xml", "/home/robocomp/robocomp/components/robocomp-ursus/etc/ursus-errors.xml", stdDev_T, 0, 0)









