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

import sys, os, Ice, traceback
import time


from PySide import *
from genericworker import *
from PyQt4 import QtGui

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	pass
if len(ROBOCOMP)<1:
	print 'ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"JointMotor.ice")
from RoboCompJointMotor import *

#------------------------------------------------------#
#         VARIABLES GLOBALES DEL PROGRAMA              #
#------------------------------------------------------#
posicionCubos = dict()

#------------------------------------------------------#
#          CLASES Y METODOS DEL PROGRAMA               #
#------------------------------------------------------#
class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
		
		# BRAZO IZQUIERDO
		self.ui.botonArriba_L.clicked.connect(self.arriba_L)
		self.ui.botonAbajo_L.clicked.connect(self.abajo_L)
		# BRAZO DERECHO
		self.ui.botonArriba_R.clicked.connect(self.arriba_R)
		self.ui.botonAbajo_R.clicked.connect(self.abajo_R)
		# MANO DERECHA:
		self.ui.botonCerrar.clicked.connect(self.cerrar)
		self.ui.botonAbrir.clicked.connect(self.abrir)
		
		# BRAZO DERECHO ENTERO
		self.ui.botonBrazo.clicked.connect(self.brazoDereho)
		
		# Pueba compleja
		self.ui.botonCargar.clicked.connect(self.cargarCubos)
		self.ui.botonIR1.clicked.connect(self.BIK)
		self.ui.botonIR2.clicked.connect(self.BIK)
		self.ui.botonIR3.clicked.connect(self.BIK)
		


	def setParams(self, params):
		try:
			par = params["BIK"]
			innermodel_path=par.value
			print "Cargado fichero XML..."
		#	innermodel = InnerModel(innermodel_path)
		except:
			tracebak.print_exc()
			print "Error reading config params"
		return True

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'
		#try:
		#	differentialrobot_proxy.setSpeed(100, 0)
		#except Ice.Exception, e:
		#	tracebak.print_exc()
		#	print e
		return True

	########################################################################################################
	########################################################################################################
	@QtCore.Slot()
	def arriba_L(self):
		mapa = {'leftShoulder1':0.5, 'leftShoulder2':.70, 'leftShoulder3':-.50 , 'leftElbow':-1.30 , 'leftForeArm':.689 , 'leftWrist1':.10 , 'leftWrist2':-.0409, 'head_yaw_joint':-0.40, 'head_pitch_joint':0.20}
		for motor in mapa:
			goal = MotorGoalPosition()
			goal.position = mapa[motor]
			goal.name = motor
			goal.maxSpeed = 0.5
			self.jointmotor_proxy.setPosition(goal)

	@QtCore.Slot()
	def abajo_L(self):
		mapa = {'leftShoulder1':0.0, 'leftShoulder2':0.0, 'leftShoulder3':0.0 , 'leftElbow':0.0 , 'leftForeArm':0.0 , 'leftWrist1':0.0 , 'leftWrist2':0.0, 'head_yaw_joint':0.0, 'head_pitch_joint':0.0}
		for motor in mapa:
			goal = MotorGoalPosition()
			goal.position = mapa[motor]
			goal.name = motor
			goal.maxSpeed = 0.5
			self.jointmotor_proxy.setPosition(goal)
			
	@QtCore.Slot()
	def arriba_R(self):
		mapa = {'rightShoulder1':-0.5, 'rightShoulder2':-0.70, 'rightShoulder3':.50 , 'rightElbow':1.30 , 'rightForeArm':-.689, 'head_yaw_joint':0.30, 'head_pitch_joint':0.20}
		for motor in mapa:
			goal = MotorGoalPosition()
			goal.position = mapa[motor]
			goal.name = motor
			goal.maxSpeed = 0.5
			self.jointmotor_proxy.setPosition(goal)

	@QtCore.Slot()
	def abajo_R(self):
		mapa = {'rightShoulder1':0.0, 'rightShoulder2':0.0, 'rightShoulder3':0.0 , 'rightElbow':0.0 , 'rightForeArm':0.0, 'head_yaw_joint':0.0, 'head_pitch_joint':0.0}
		for motor in mapa:
			goal = MotorGoalPosition()
			goal.position = mapa[motor]
			goal.name = motor
			goal.maxSpeed = 0.5
			self.jointmotor_proxy.setPosition(goal)
			
	@QtCore.Slot()
	def abrir(self):
		mapa = {'rightFinger1':-0.68 , 'rightFinger2':0.5}
		for motor in mapa:
			goal = MotorGoalPosition()
			goal.position = mapa[motor]
			goal.name = motor
			goal.maxSpeed = 0.5
			self.jointmotor_proxy.setPosition(goal)
		
	@QtCore.Slot()
	def cerrar(self):
		mapa = {'rightFinger1':-1 , 'rightFinger2':1}
		for motor in mapa:
			goal = MotorGoalPosition()
			goal.position = mapa[motor]
			goal.name = motor
			goal.maxSpeed = 0.5
			self.jointmotor_proxy.setPosition(goal)
			
	@QtCore.Slot()
	def brazoDereho(self):
		self.abajo_R()
		time.sleep(2)
		
		self.arriba_R()
		time.sleep(2)
		
		self.abrir()
		time.sleep(1)
		
		self.cerrar()
		time.sleep(2)
		
		self.abajo_R()

	########################################################################################################
	########################################################################################################
	@QtCore.Slot()
	def cargarCubos(self):
		# Leemos del fichero de innermodel.xml todas las lineas y sacamos las 
		# lineas que coincidan con el patron transform id="CUBO_"
		print 'CARGANDO CUBOS...'
		# Inicializamos el fichero de lectura
		infile = open("/home/robocomp/robocomp/files/innermodel/worlds/pruebaRockin.xml", 'r') 
		for line in infile:
			if line.find('<transform id="CUBO_')!=-1:
				# Tenemos la linea con el patron: <transform id="CUBO_X" tx="X" ty="X" tz="X" [rx, ry rz]> 
				# Inicializamos la pose del cubo:
				pose=[0,0,0,0,0,0]
				
				# dividimos por '>' y despues por espacios en blanco=>['<transform', 'id="CUBO_X"', 'tx="X"', 'ty="X"', 'tz="X">', 'rx="X"', 'ry="X"', 'rz="X"']
				# Longitud maxima: 8 (con rotaciones)
				# Longitud minima: 5 (solo traslaciones)
				vector = (line.split('>'))[0].split()
				# Sacamos el nombre del cubo:
				nombreCubo = str((vector[1].split('='))[1].split('"')[1])
				
				# Sacamos las tralaciones y las rotaciones eliminando todo lo que no sea numerico
				for elemento in vector:
					if elemento.find('tx')!=-1: pose[0] = float((elemento.replace('tx="', '')).replace('"',''))
					if elemento.find('ty')!=-1: pose[1] = float((elemento.replace('ty="', '')).replace('"',''))
					if elemento.find('tz')!=-1: pose[2] = float((elemento.replace('tz="', '')).replace('"',''))
					if elemento.find('rx')!=-1: pose[3] = float((elemento.replace('rx="', '')).replace('"',''))
					if elemento.find('ry')!=-1: pose[4] = float((elemento.replace('ry="', '')).replace('"',''))
					if elemento.find('rz')!=-1: pose[5] = float((elemento.replace('rz="', '')).replace('"',''))
					
				# Guardamos la pose en el dicionario
				posicionCubos[nombreCubo] = pose
		infile.close()

		i=1
		for cubo in posicionCubos:
			if i==1: 
				self.ui.labelCubo1.setText(str(cubo)+': '+str(posicionCubos[cubo]))
				self.ui.botonIR1.setEnabled(True)
			if i==2: 
				self.ui.labelCubo2.setText(str(cubo)+': '+str(posicionCubos[cubo]))
				self.ui.botonIR2.setEnabled(True)
			if i==3: 
				self.ui.labelCubo3.setText(str(cubo)+': '+str(posicionCubos[cubo]))
				self.ui.botonIR3.setEnabled(True)
			i = i+1

			
		print 'FIN CARGAR CUBOS: '
		
		@QtCore.Slot()
		def BIK(self):
			# LLamar al BIK y pasarle el vector POSE:
			


