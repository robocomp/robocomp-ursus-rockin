import sys, os, Ice, traceback, math, random, copy
import numpy as np
import commands
import time
import subprocess
from PySide import *
from PyQt4 import QtGui
from PyQt4.QtGui import QApplication, QDialog
from ui_MainWindow import *


ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print '|| ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"InverseKinematics.ice")
from RoboCompInverseKinematics import *

import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

ic = None

class Auxiliar(QtGui.QDialog,Ice.Application):
	#-------------------------------------------------------------------
	# CONSTRUCTOR: inicializa la clase y levanta la interfaz de usuario
	#-------------------------------------------------------------------
	def __init__(self, params):
		super(Auxiliar, self).__init__()
				
		global ic
	  	#ic = self.communicator()
		ic = Ice.initialize(params)
	  	print "---->",type(ic)
	  	
		self.inversekinematics_proxy = None
		
		self.ui = Ui_guiDlg()
		self.ui.setupUi(self)
		self.show()
		
		self.stop = False
		self.ui.stopButton.clicked.connect(self.stopTest)
		self.ui.testButton.clicked.connect(self.runTest)
		
	#------------------------------------------------------------------#
	#                 METODOS AUXILIARES DEL PROGRAMA                  #
	#------------------------------------------------------------------#
	#######################################################
	### SLOTS DE LA CLASE
	#######################################################
	def stopTest(self):
		if self.stop == True:
			self.stop = False
			self.ui.stopButton.setText("Reanude")
		else:
			self.stop = True
			self.ui.stopButton.setText("Stop")

	#### TODO QUITAR COMENTARIOS 
	def runTest(self):
		self.ui.testButton.setEnabled(False)
		self.weights = WeightVector() #vector de pesos
		self.weights.x = 1
		self.weights.y = 1
		self.weights.z = 1
		self.weights.rx = 0.1
		self.weights.ry = 0.1
		self.weights.rz = 0.1
		
		self.axis = Axis() #vector de ejes
		self.axis.x = 0
		self.axis.y = 0
		self.axis.z = 1
			
		#Genero 100 targets aleatorios y los almaceno en un vector:
		self.targets = []
		while len(self.targets)<100:
			pose6D    = Pose6D()
			pose6D.x  = random.randint(140, 300)
			pose6D.y  = random.randint(780, 800)
			pose6D.z  = random.randint(300, 390)
			pose6D.rx = 0
			pose6D.ry = -0.80
			pose6D.rz = -3.1416
			if (pose6D in self.targets) == False:
				self.targets.append(pose6D)
				
		#Eliminamos los ficheros que puedan contener basura:
		#os.system("rm /home/robocomp/robocomp/components/robocomp-ursus/components/inversekinematics/data.txt")
		os.system("rm /home/robocomp/robocomp/components/robocomp-ursus/components/visualik/data.txt")
		
		#Variables del bucle:
		self.init_value = 5.0 #0.0
		self.end_value  = 50.00#0.0
		self.step_value = 5
		self.i = 2 #change to 1

		self.stdDev_T = self.init_value
		self.testTimer = QtCore.QTimer()
		self.testTimer.timeout.connect(self.doTest)
		self.testTimer.start(1)
		
	#### TODO QUITAR COMENTARIOS	
	def doTest(self):
		if self.stdDev_T >= self.end_value:
			self.testTimer.stop()
			os.system("rm /home/robocomp/robocomp/components/robocomp-ursus/components/inversekinematics/data.txt")
			self.ui.testButton.setEnabled(True)
			
		self.ui.errorLabel.setText('Running experiment with error in translation: stdDev_T='+str(self.stdDev_T))
		print "Error: ", self.stdDev_T
		
		os.system('killall -9 ursuscommonjointcomp apriltagscomp inversekinematics VisualBIK')
		self.generateErrorsXML("/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/visualBIKexperiment/ursus.xml", "/home/robocomp/robocomp/components/robocomp-ursus-rockin/files/visualBIKexperiment/ursus_errors.xml", self.stdDev_T, 0, 0)
		
		##LEVANTAMOS EL URSUS COMMON JOINT
		self.ui.textEdit_2.append(str(self.i)+'---> ejecutando ursus common joint\n')
		print '############################# ejecutando ursus common joint'
		os.system('killall -9 ursuscommonjointcomp')
		os.system('nohup /home/robocomp/robocomp/components/robocomp-ursus/components/ursusCommonJoint/bin/ursuscommonjointcomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/ficheros_Test_VisualBIK/ursusCommon.conf > /dev/null &')
		##LEVANTAMOS EL APRIL TAGS
		self.ui.textEdit_2.append(str(self.i)+'---> ejecutando apriltags\n')
		print '############################# ejecutando apriltags'
		os.system('killall -9 apriltagscomp')
		os.system("nohup /home/robocomp/robocomp/components/robocomp-robolab/components/apriltagsComp/bin/apriltagscomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/ficheros_Test_VisualBIK/apriltags.conf > /dev/null &")
		#DORMIMOS 5 SEGUNDOS
		time.sleep(5)
		
		##LEVANTAMOS EL INVERSEKINEMATICS
		self.ui.textEdit_2.append(str(self.i)+'--->  ejecutando IK\n')
		print '############################# ejecutando IK'
		os.system('killall -9 inversekinematics')
		os.system('nohup /home/robocomp/robocomp/components/robocomp-ursus/components/inversekinematics/bin/inversekinematics --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/ficheros_Test_VisualBIK/ikSim.conf > /dev/null &')
		#DORMIMOS 5 SEGUNDOS
		time.sleep(5)
		
		##LEVANTAMOS EL INVERSEKINEMATICSGRAPH
		#self.ui.textEdit_2.append(str(self.i)+'--->  ejecutando GIK\n')
		#print '############################# ejecutando GIK'
		#os.system('killall -9 ikGraphGenerator')
		#os.system('nohup /home/robocomp/robocomp/components/robocomp-ursus/components/ikGraphGenerator/bin/ikGraphGenerator --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/ficheros_Test_VisualBIK/ikgSim.conf 2> graphDATA.tx &')
		#DORMIMOS 5 SEGUNDOS
		#time.sleep(7)		
		
		##LEVANTAMOS EL VISUAL INVERSEKINEMATICS
		self.ui.textEdit_2.append(str(self.i)+'--->  ejecutando VIK\n')
		print '############################# ejecutando VIK'
		os.system('killall -9 VisualBIK')
		os.system('nohup gdb /home/robocomp/robocomp/components/robocomp-ursus/components/visualik/bin/VisualBIK --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/ficheros_Test_VisualBIK/vikSim.conf 2> visualDATA.txt &')
		#DORMIMOS 5 SEGUNDOS
		time.sleep(5)
		
		#### CREAR EL PROXY AL INVERSEKINEMATICS
		self.initializeProxy()
		##ENVIAMOS LOS TARGETS:
		j=0
		for pose in self.targets:
			self.ui.poseLabel.setText("Pose Actual: ["+str(pose.x)+", "+str(pose.y)+", "+str(pose.z)+", "+str(pose.rx)+", "+str(pose.ry)+", "+str(pose.rz)+"]")
			print "Pose Actual: ["+str(pose.x)+", "+str(pose.y)+", "+str(pose.z)+", "+str(pose.rx)+", "+str(pose.ry)+", "+str(pose.rz)+"]"
			try:
				#Primero a la CABEZA para que mire:
				part = "HEAD"
				self.inversekinematics_proxy.setTargetAlignaxis(part, pose, self.axis)
				
				part = "RIGHTARM"
				identificador = self.inversekinematics_proxy.setTargetPose6D(part,pose, self.weights)
				
				state = TargetState()
				state = self.inversekinematics_proxy.getTargetState("RIGHTARM", identificador)
				while state.finish!=True:
					state = self.inversekinematics_proxy.getTargetState("RIGHTARM", identificador)
					
				#Ya hemos terminado: escribimos el dato
				infile = open ("/home/robocomp/robocomp/components/robocomp-ursus/components/visualik/data.txt" ,"r" ) 
				lines = infile.readlines () 
				if len(lines)<=0:
                                    print "FICHERO VACIO"
                                    sys.exit(-1)
                                    
				infile.close () 
				last_line = lines [ len ( lines ) -1 ] 
				self.ui.textEdit.append(last_line+'\n')
				step1 = last_line.split(":");
				step2 = step1[2].split(" ")
				
				error_vt = float(step2[0])
				print "Error visual alcanzado: ", error_vt 
				
			except:
				print "EXCEPCION EN SEND POSE 6D"
				traceback.print_exc()

	
		print 'ITERACION HECHA!'
		#GUARDAMOS LOS DATOS EN OTRO FICHERO
		os.system('mv /home/robocomp/robocomp/components/robocomp-ursus/components/visualik/data.txt /home/robocomp/robocomp/components/robocomp-ursus-rockin/files/visualBIKexperiment/datosObtenidos_'+str(self.i).zfill(5)+'.txt')
		
		self.stdDev_T += self.step_value
		self.i += 1
				
	#######################################################
	### METODOS PRIVADOS DE LA CLASE
	#######################################################
	#### Inicializa el proxy al visualIK.
	def initializeProxy(self):
		print "PROXY"
		import RoboCompInverseKinematics
		status = 0
		try:
			# Remote object connection for InverseKinematics
			try:
				proxyString = ic.getProperties().getProperty('InverseKinematicsProxy')
				try:
					basePrx = ic.stringToProxy(proxyString)
					self.inversekinematics_proxy = RoboCompInverseKinematics.InverseKinematicsPrx.checkedCast(basePrx)
				except Ice.Exception:
					print 'Cannot connect to the remote object (InverseKinematics)', proxyString
					status = 1
			except Ice.Exception, e:
				print e
				print 'Cannot get InverseKinematicsProxy property.'
				status = 1
		except:
			traceback.print_exc()
			status = 1
			
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
					lineOut = self.cambiaError(line, fields)
				elif "<translation" in line:
					lineOut = self.cambiaError(line, fields)
				elif "<rotation" in line:
					lineOut = self.cambiaError(line, fields)
				elif "<rgbd" in line:
					lineOut = self.cambiaError(line, fields)
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
								err = np.random.normal(0., fields[fl], 1)[0]
								while err > 2.*fields[fl]:
									err = np.random.normal(0., fields[fl], 1)[0]
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
