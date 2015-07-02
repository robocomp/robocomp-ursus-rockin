#
# Copyright (C) 2015 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import sys, os, Ice, traceback, math, random
import numpy as np
import commands
import time
import subprocess
from PySide import *
from genericworker import *

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print 'genericworker.py: ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"JointMotor.ice")
from RoboCompJointMotor import *
Ice.loadSlice(preStr+"InverseKinematics.ice")
from RoboCompInverseKinematics import *



class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
		
		self.stop = False
		self.ui.stopButton.clicked.connect(self.stopTest)
		self.ui.testButton.clicked.connect(self.runTest)

	def setParams(self, params):
		#try:
		#	par = params["InnerModelPath"]
		#	innermodel_path=par.value
		#	innermodel = InnerModel(innermodel_path)
		#except:
		#	traceback.print_exc()
		#	print "Error reading config params"
		return True

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'
		#try:
		#	differentialrobot_proxy.setSpeed(100, 0)
		#except Ice.Exception, e:
		#	traceback.print_exc()
		#	print e
		return True
	
	#######################################################
	### SLOTS DE LA CLASE
	#######################################################
	@QtCore.Slot()
	def stopTest(self):
		if self.stop == True:
			self.stop = False
			self.ui.stopButton.setText("Reanude")
		else:
			self.stop = True
			self.ui.stopButton.setText("Stop")
		
	@QtCore.Slot()
	def runTest(self):
		self.ui.testButton.setEnabled(False)
		import RoboCompInverseKinematics
		weights = RoboCompInverseKinematics.WeightVector() #vector de pesos
		weights.x = 1
		weights.y = 1
		weights.z = 1
		weights.rx = 0.1
		weights.ry = 0.1
		weights.rz = 0.1
			
		#Genero 100 targets aleatorios y los almaceno en un vector:
		targets = []
		while len(targets)<101:
			pose6D    = RoboCompInverseKinematics.Pose6D()
			pose6D.x  = random.randint(140, 300)
			pose6D.y  = random.randint(780, 850)
			pose6D.z  = random.randint(370, 400)
			pose6D.rx = 0
			pose6D.ry = 0
			pose6D.rz = 3.1416
			if (pose6D in targets) == False:
				targets.append(pose6D)
				
		#Eliminamos los ficheros que puedan contener basura:
		os.system("rm /home/robocomp/robocomp/components/robocomp-ursus/components/inversekinematics/data.txt")
		os.system("rm /home/robocomp/robocomp/components/robocomp-ursus/components/visualik/data.txt")
		
		#Variables del bucle:
		init_value = 30.0 #0.0
		end_value = 30.0
		step_value = 3
		i = 1
		for stdDev_T in np.arange(init_value, end_value+0.0001, step_value):	
			self.ui.errorLabel.setText('Running experiment with error in translation: stdDev_T='+str(stdDev_T))
			
			os.system('killall -9 ursuscommonjointcomp apriltagscomp inversekinematics VisualBIK')
			self.generateErrorsXML("/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/visualBIKexperiment/ursus.xml", "/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/visualBIKexperiment/ursus_errors.xml", stdDev_T, 0, 0)
			
			#LEVANTAMOS EL URSUS COMMON JOINT
			self.ui.textEdit_2.setText(str(i), str(x), '############################# ejecutando ursus common joint')
			os.system('killall -9 ursuscommonjointcomp')
			os.system('nohup /home/robocomp/robocomp/components/robocomp-ursus/components/ursusCommonJoint/bin/ursuscommonjointcomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/ficheros_Test_VisualBIK/ursusCommon.conf > /dev/null &')
			#LEVANTAMOS EL APRIL TAGS
			self.ui.textEdit_2.setText(str(i), str(x), '############################# ejecutando apriltags')
			os.system('killall -9 apriltagscomp')
			os.system("nohup /home/robocomp/robocomp/components/robocomp-robolab/components/apriltagsComp/bin/apriltagscomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/ficheros_Test_VisualBIK/apriltags.conf > /dev/null &")
			#DORMIMOS 5 SEGUNDOS
			time.sleep(5)
			
			##LEVANTAMOS EL INVERSEKINEMATICS
			self.ui.textEdit_2.setText(str(i), str(x), '############################# ejecutando IK')
			os.system('killall -9 inversekinematics')
			os.system('nohup /home/robocomp/robocomp/components/robocomp-ursus/components/inversekinematics/bin/inversekinematics --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus/components/inversekinematics/etc/configdefinitivo > /dev/null &')
			#DORMIMOS 5 SEGUNDOS
			time.sleep(5)
			
			##LEVANTAMOS EL VISUAL INVERSEKINEMATICS
			self.ui.textEdit_2.setText(str(i), str(x), '############################# ejecutando VIK')
			os.system('killall -9 VisualBIK')
			os.system('nohup /home/robocomp/robocomp/components/robocomp-ursus/components/visualik/bin/VisualBIK --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus/components/visualik/etc/configDefinitivo > /dev/null &')
			#DORMIMOS 5 SEGUNDOS
			time.sleep(5)
			
			##ENVIAMOS LOS TARGETS:
			for pose in targets:
				self.ui.poseLabel.setText("Pose Actual: ["+str(pose.x)+", "+str(pose.y)+", "++str(pose.z)+", "+str(pose.rx)+", "+str(pose.ry)+", "+str(pose.rz)+", ")
				try:
					part = "RIGHTARM"
					identificador = self.inversekinematics_proxy.setTargetPose6D(part,pose, weights)
					
					state = RoboCompInverseKinematics.TargetState()
					state = self.inversekinematics_proxy.getTargetState("RIGHTARM", identificador)
					while state.finish!=True:
						state = self.inversekinematics_proxy.getTargetState("RIGHTARM", identificador)
				except:
					print "EXCEPCION EN SEND POSE 6D"
		
			print 'ITERACION HECHA!'
			#GUARDAMOS LOS DATOS EN OTRO FICHERO
			os.system('mv /home/robocomp/robocomp/components/robocomp-ursus/components/visualik/data.txt /home/robocomp/robocomp/components/robocomp-ursus-rockin/files/visualBIKexperiment/datosObtenidos_'+str(i).zfill(5)+'.txt')
			i += 1
		#FIN DEL FOR
		os.system("rm /home/robocomp/robocomp/components/robocomp-ursus/components/inversekinematics/data.txt")
		self.ui.testButton.setEnabled(True)
				
	#######################################################
	### METODOS PRIVADOS DE LA CLASE
	#######################################################
	def generateErrorsXML(self, input_file, output_file, stdDevT, stdDevR, stdDevF):
		fields = dict()
		fields["tx"]    = stdDevT
		fields["ty"]    = stdDevT
		fields["tz"]    = stdDevT
		fields["rx"]    = stdDevR
		fields["ry"]    = stdDevR
		fields["rz"]    = stdDevR
		fields["focal"] = stdDevF

		entrada = open(input_file, "r")
		salida  = open(output_file, "w")

		for line in entrada.readlines():
			lineOut = copy.deepcopy(line)
			if "@error".lower() in line.lower():
				if "<transform" in line:
					lineOut = cambiaError(line, fields)
				elif "<translation" in line:
					lineOut = cambiaError(line, fields)
				elif "<rotation" in line:
					lineOut = cambiaError(line, fields)
				elif "<rgbd" in line:
					lineOut = cambiaError(line, fields)
				else:
					print 'whaaaat', line
			salida.write(lineOut)
		entrada.close()
		salida.close()
		
	def cambiaError(self, line, fields):
		ret = ''
		s = line.split('"')
		first = ''
		for i in xrange(len(s)):
			if i%2 == 1:
				done = False
				for field in fields:
					if s[i-1].endswith(field+'='):
						if len(s)>i and "@error".lower() in s[i].lower():
							XXX = s[i]
							base = s[i].lower().split("@error")[0]
							if base == '':
								base = 0
							else:
								base = float(base)
							fl = s[i-1][:-1].strip()
							#print 'field', fl
							err = 0
							if fields[fl] > 0.:
								err = numpy.random.normal(0., fields[fl], 1)[0]
								while err > 2.*fields[fl]:
									err = numpy.random.normal(0., fields[fl], 1)[0]
							ret += first + str(base+err)
							first = '"'
							done = True
							break
				if not done:
					ret += first + s[i]
					first = '"'
			else:
				ret += first + s[i]
				first = '"'
		return ret






