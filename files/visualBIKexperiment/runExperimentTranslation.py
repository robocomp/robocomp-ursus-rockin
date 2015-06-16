from generateErrors import *

import numpy as np
import commands
import time, os
import subprocess

init_value = 0.0
end_value = 50.0
step_value = 0.5

i = 1

os.system("rm /home/robocomp/robocomp/components/robocomp-ursus/components/visualbik/datosObtenidos.txt")

for stdDev_T in np.arange(init_value, end_value+0.0001, step_value):
	
	print 'Running experiment with error in translation: stdDev_T='+str(stdDev_T)
	for x in xrange(250):
		os.system('killall -9 ursuscommonjointcomp apriltagscomp inversekinematics VisualBIK')
		generateErrorsXML("ursus.xml", "/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/visualBIKexperiment/ursus_errors.xml", stdDev_T, 0, 0)

		#LEVANTAMOS EL URSUS COMMON JOINT
		print i, x, '############################# ejecutando ursus common joint'
		os.system('killall -9 ursuscommonjointcomp')
		os.system('nohup /home/robocomp/robocomp/components/robocomp-ursus/components/ursusCommonJoint/bin/ursuscommonjointcomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/ficheros_Test_VisualBIK/ursusCommon.conf > /dev/null &')

		#LEVANTAMOS EL APRIL TAGS
		print i, x, '############################# ejecutando apriltags'
		os.system('killall -9 apriltagscomp')
		os.system("nohup /home/robocomp/robocomp/components/robocomp-robolab/components/apriltagsComp/bin/apriltagscomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/ficheros_Test_VisualBIK/apriltags.conf > /dev/null &")

		#DORMIMOS 5 SEGUNDOS
		time.sleep(5)
		
		##LEVANTAMOS EL INVERSEKINEMATICS
		print i, x, '############################# ejecutando IK'
		os.system('killall -9 inversekinematics')
		os.system('nohup /home/robocomp/robocomp/components/robocomp-ursus/components/inversekinematics/bin/inversekinematics --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus/components/inversekinematics/etc/configDefinitivo > /dev/null &')

		#DORMIMOS 5 SEGUNDOS
		time.sleep(5)
		
		##LEVANTAMOS EL VISUAL INVERSEKINEMATICS
		print i, x, '############################# ejecutando VIK'
		os.system('killall -9 VisualBIK')
		os.system('nohup /home/robocomp/robocomp/components/robocomp-ursus/components/visualbik/bin/VisualBIK --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus/components/visualbik/etc/configDefinitivo > /dev/null &')
		#DORMIMOS 5 SEGUNDOS
		time.sleep(5)
		
		##LEVANTAMOS EL MUEVE BRAZO
		print i, x, '############################# ejecutando M'
		os.system('pkill -9 name.py')
		os.system('python /home/robocomp/robocomp/components/robocomp-ursus-rockin/components/mueveBrazo/src/name.py /home/robocomp/robocomp/components/robocomp-ursus-rockin/components/mueveBrazo/etc/configDefinitivo')


		#os.system('killall -9 ursuscommonjointcomp apriltagscomp inversekinematics VisualBIK')
		print x*10

	print 'hecho!'
	os.system('mv /home/robocomp/robocomp/components/robocomp-ursus/components/visualbik/datosObtenidos.txt /home/robocomp/robocomp/components/robocomp-ursus/components/visualbik/datosObtenidos_'+str(i).zfill(5)+'.txt')
	i += 1
	
	
	
	