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

import sys, os, Ice, traceback, time
import random

from PySide import *
from genericworker import *

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
		print "HOLA DESDE EL CONSTRUCTOR DEL SPECIFIC"

		super(SpecificWorker, self).__init__(proxy_map)

		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)

		self.ui.txsb.setValue(0)
		self.ui.tysb.setValue(800)
		self.ui.tzsb.setValue(400)
		self.ui.rxsb.setValue(0)
		self.ui.rysb.setValue(0)
		self.ui.rzsb.setValue(3.1415)

		# BRAZO IZQUIERDO
		#self.ui.botonArriba_L.clicked.connect(self.arriba_L)
		#self.ui.botonAbajo_L.clicked.connect(self.abajo_L)
		# BRAZO DERECHO
		#self.ui.botonArriba_R.clicked.connect(self.arriba_R)
		#self.ui.botonAbajo_R.clicked.connect(self.abajo_R)
		# MANO DERECHA:
		#self.ui.botonCerrar.clicked.connect(self.cerrar)
		#self.ui.botonAbrir.clicked.connect(self.abrir)
		# BRAZO DERECHO ENTERO
		#self.ui.botonBrazo.clicked.connect(self.brazoDereho)

		#Prueba visualBIK
		self.ui.botonRotacion.clicked.connect(self.movimiento_con_Rotacion)
		self.ui.botonTraslacion.clicked.connect(self.movimiento_sin_Rotacion)
		self.ui.botonHome.clicked.connect(self.abajo_R)

		#Prueba 2000 puntos
		self.ui.entrenamientoButton.clicked.connect(self.prueba2000puntos)
		
		# Pueba compleja
	#	self.ui.botonCargar.clicked.connect(self.cargarCubos)
	#	self.ui.botonIR1.clicked.connect(self.llamarBIK_1)
	#	self.ui.botonIR2.clicked.connect(self.llamarBIK_2)
	#	self.ui.botonIR3.clicked.connect(self.llamarBIK_3)

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


	########################################################################################################
	########################################################################################################
	#@QtCore.Slot()
	#def arriba_L(self):
		#mapa = {'leftShoulder1':0.5, 'leftShoulder2':.70, 'leftShoulder3':-.50 , 'leftElbow':-1.30 , 'leftForeArm':.689 , 'leftWrist1':.10 , 'leftWrist2':-.0409, 'head_yaw_joint':-0.40, 'head_pitch_joint':0.20}
		#for motor in mapa:
			#goal = MotorGoalPosition()
			#goal.position = mapa[motor]
			#goal.name = motor
			#goal.maxSpeed = 0.5
			#try:
				#self.jointmotor_proxy.setPosition(goal)
			#except CollisionException:
				#print "Error en arriba_L: ",CollisionException

	@QtCore.Slot()
	def abajo_L(self):
		mapa = {'leftShoulder1':0.1, 'leftShoulder2':0.1, 'leftShoulder3':-0.1 , 'leftElbow':-0.15 , 'leftForeArm':0.0 , 'leftWrist1':0.0 , 'leftWrist2':0.0, 'head_yaw_joint':0.0, 'head_pitch_joint':0.0}
		for motor in mapa:
			goal = MotorGoalPosition()
			goal.position = mapa[motor]
			goal.name = motor
			goal.maxSpeed = 0.5
			try:
				self.jointmotor_proxy.setPosition(goal)
			except CollisionException:
				print "Error en abajo_L: ",CollisionException

	#@QtCore.Slot()
	#def arriba_R(self):
		#mapa = {'rightShoulder1':-0.5, 'rightShoulder2':-0.70, 'rightShoulder3':.50 , 'rightElbow':1.30 , 'rightForeArm':-.689, 'head_yaw_joint':0.30, 'head_pitch_joint':0.20}
		#for motor in mapa:
			#goal = MotorGoalPosition()
			#goal.position = mapa[motor]
			#goal.name = motor
			#goal.maxSpeed = 0.5
			#try:
				#self.jointmotor_proxy.setPosition(goal)
			#except CollisionException:
				#print "Error en arriba_R: ",CollisionException

	@QtCore.Slot()
	def abajo_R(self):
		mapa = {'rightShoulder1':-0.1, 'rightShoulder2':-0.1, 'rightShoulder3':0.1 , 'rightElbow':0.15 , 'rightForeArm':0.1, 'rightWrist1':0.0 , 'rightWrist2':0.0, 'head_yaw_joint':0.0, 'head_pitch_joint':0.85, 'leftShoulder1':0.1, 'leftShoulder2':0.1, 'leftShoulder3':-0.1 , 'leftElbow':-0.1 , 'leftForeArm':0.0 , 'leftWrist1':0.0 , 'leftWrist2':0.0}
		for motor in mapa:
			goal = MotorGoalPosition()
			goal.position = mapa[motor]
			goal.name = motor
			goal.maxSpeed = 0.5
			try:
				self.jointmotor_proxy.setPosition(goal)
			except CollisionException:
				print "Error en abajo_R: ", CollisionException

	#@QtCore.Slot()
	#def abrir(self):
		#mapa = {'rightFinger1':-0.68 , 'rightFinger2':0.5}
		#for motor in mapa:
			#goal = MotorGoalPosition()
			#goal.position = mapa[motor]
			#goal.name = motor
			#goal.maxSpeed = 0.5
			#try:
				#self.jointmotor_proxy.setPosition(goal)
			#except CollisionException:
				#print "Error en abrir Finger: ",CollisionException

	#@QtCore.Slot()
	#def cerrar(self):
		#mapa = {'rightFinger1':-1 , 'rightFinger2':1}
		#for motor in mapa:
			#goal = MotorGoalPosition()
			#goal.position = mapa[motor]
			#goal.name = motor
			#goal.maxSpeed = 0.5
			#try:
				#self.jointmotor_proxy.setPosition(goal)
			#except CollisionException:
				#print "Error en cerrar Finger: ", CollisionException

	#@QtCore.Slot()
	#def brazoDereho(self):
		#self.abajo_R()
		#time.sleep(2)

		#self.arriba_R()
		#time.sleep(2)

		#self.abrir()
		#time.sleep(1)

		#self.cerrar()
		#time.sleep(2)

		#self.abajo_R()

	########################################################################################################
	########################################################################################################
	@QtCore.Slot()
	def movimiento_con_Rotacion(self):
		# LLamar al BIK y pasarle el vector POSE:
		#print 'Preparando vector pose 6D'
		import RoboCompBodyInverseKinematics
		pose6D = RoboCompBodyInverseKinematics.Pose6D() #target al que se movera
		pose6D.x  = float(self.ui.txsb.value())
		pose6D.y  = float(self.ui.tysb.value())
		pose6D.z  = float(self.ui.tzsb.value())
		pose6D.rx = float(self.ui.rxsb.value())
		pose6D.ry = float(self.ui.rysb.value())
		pose6D.rz = float(self.ui.rzsb.value())
		print 'Llamando a BIK con pose6D: ',pose6D

		weights = RoboCompBodyInverseKinematics.WeightVector() #vector de pesos
		weights.x = 1
		weights.y = 1
		weights.z = 1
		weights.rx = 1
		weights.ry = 1
		weights.rz = 1
		
		axis = RoboCompBodyInverseKinematics.Axis()
		axis.x = 0
		axis.y = 0
		axis.z = 1

		radius = 150 #radio
		try:
			part = "RIGHTARM"
			self.bodyinversekinematics_proxy.setTargetPose6D(part,pose6D, weights, radius)
			#part = "HEAD"
			#self.bodyinversekinematics_proxy.pointAxisTowardsTarget(part, pose6D, axis, False, 0)
		except Ice.Exception, e:
			print "Error en movimiento_con_Rotacion", e


	@QtCore.Slot()
	def movimiento_sin_Rotacion(self):
		# LLamar al BIK y pasarle el vector POSE:
		import RoboCompBodyInverseKinematics
		print 'Preparando vector pose 6D'
		pose6D = RoboCompBodyInverseKinematics.Pose6D() #target al que se movera
		pose6D.x  = float(self.ui.txsb.value())
		pose6D.y  = float(self.ui.tysb.value())
		pose6D.z  = float(self.ui.tzsb.value())
		pose6D.rx = float(self.ui.rxsb.value())
		pose6D.ry = float(self.ui.rysb.value())
		pose6D.rz = float(self.ui.rzsb.value())
		print 'Llamando a BIK con pose6D: ',pose6D

		weights = RoboCompBodyInverseKinematics.WeightVector() #vector de pesos
		weights.x = 1
		weights.y = 1
		weights.z = 1
		weights.rx = 0
		weights.ry = 0
		weights.rz = 0
				
		axis = RoboCompBodyInverseKinematics.Axis()
		axis.x = 0
		axis.y = 0
		axis.z = 1

		radius = 150 #radio
		try:
			part = "RIGHTARM"
			self.bodyinversekinematics_proxy.setTargetPose6D(part,pose6D, weights, radius)
			#part = "HEAD"
			#self.bodyinversekinematics_proxy.pointAxisTowardsTarget(part, pose6D, axis, False, 0)
		except:
			print "Error en movimiento_sin_Rotacion"
			
	########################################################################################################
	########################################################################################################
	@QtCore.Slot()
	def prueba2000puntos(self):
		# Entre un rango de 100 - 200 en X
		#                   800 - 900 en Y
		#                   400 - 200 en Z
		import RoboCompBodyInverseKinematics
		for i in range(0, 100):
			print 'i: ',i
			pose6D = RoboCompBodyInverseKinematics.Pose6D() #target al que se movera
			pose6D.x  = random.uniform(100.0, 200.0)
			pose6D.y  = random.uniform(800.0, 900.0)
			pose6D.z  = random.uniform(400.0, 250.0)
			pose6D.rx = 0
			pose6D.ry = 0
			pose6D.rz = 3.1416
			#print 'Llamando a BIK con pose6D: ',pose6D
			self.ui.label_13.setText(self.ui.label_13.getText()+'\n('+str(pose6D.x)+', '+str(pose6D.y)+', '+str(pose6D.z)+'), ['+str(pose6D.rx)+', '+str(pose6D.ry)+', '+str(pose6D.rz)+']')

			weights = RoboCompBodyInverseKinematics.WeightVector() #vector de pesos
			weights.x = 1
			weights.y = 1
			weights.z = 1
			weights.rx = 1
			weights.ry = 1
			weights.rz = 1
					
			axis = RoboCompBodyInverseKinematics.Axis()
			axis.x = 0
			axis.y = 0
			axis.z = 1

			radius = 150 #radio
			try:
				part = "RIGHTARM"
				self.bodyinversekinematics_proxy.setTargetPose6D(part,pose6D, weights, radius)
				part = "HEAD"
				self.bodyinversekinematics_proxy.pointAxisTowardsTarget(part, pose6D, axis, False, 0)
			except:
				print "Error en movimiento_sin_Rotacion"

	########################################################################################################
	########################################################################################################
	#@QtCore.Slot()
	#def cargarCubos(self):
		## Leemos del fichero de innermodel.xml todas las lineas y sacamos las
		## lineas que coincidan con el patron transform id="CUBO_"
		#print 'CARGANDO CUBOS...'
		## Inicializamos el fichero de lectura
		#infile = open("/home/robocomp/robocomp/files/innermodel/worlds/pruebaRockin.xml", 'r')
		#item = 0
		#for line in infile:
			#if line.find('<transform id="CUBO_')!=-1:
				## Tenemos la linea con el patron: <transform id="CUBO_X" tx="X" ty="X" tz="X" [rx, ry rz]>
				## Inicializamos la pose del cubo:
				#pose=[0,0,0,0,0,0]

				## dividimos por '>' y despues por espacios en blanco=>['<transform', 'id="CUBO_X"', 'tx="X"', 'ty="X"', 'tz="X">', 'rx="X"', 'ry="X"', 'rz="X"']
				## Longitud maxima: 8 (con rotaciones)
				## Longitud minima: 5 (solo traslaciones)
				#vector = (line.split('>'))[0].split()
				## Sacamos el nombre del cubo:
				##nombreCubo = str((vector[1].split('='))[1].split('"')[1])
				##nombresCubos[i] = nombreCubo

				## Sacamos las tralaciones y las rotaciones eliminando todo lo que no sea numerico
				#for elemento in vector:
					#if elemento.find('tx')!=-1: pose[0] = float((elemento.replace('tx="', '')).replace('"',''))
					#if elemento.find('ty')!=-1: pose[1] = float((elemento.replace('ty="', '')).replace('"',''))
					#if elemento.find('tz')!=-1: pose[2] = float((elemento.replace('tz="', '')).replace('"',''))
					#if elemento.find('rx')!=-1: pose[3] = float((elemento.replace('rx="', '')).replace('"',''))
					#if elemento.find('ry')!=-1: pose[4] = float((elemento.replace('ry="', '')).replace('"',''))
					#if elemento.find('rz')!=-1: pose[5] = float((elemento.replace('rz="', '')).replace('"',''))

				## Guardamos la pose en el dicionario
				#posicionCubos[item] = pose
				#item = item+1
		#infile.close()

		#i=1
		#for cubo in posicionCubos:
			#if i==1:
				#self.ui.labelCubo1.setText(str(cubo)+': '+str(posicionCubos[cubo]))
				#self.ui.botonIR1.setEnabled(True)
			#if i==2:
				#self.ui.labelCubo2.setText(str(cubo)+': '+str(posicionCubos[cubo]))
				#self.ui.botonIR2.setEnabled(True)
			#if i==3:
				#self.ui.labelCubo3.setText(str(cubo)+': '+str(posicionCubos[cubo]))
				#self.ui.botonIR3.setEnabled(True)
			#i = i+1

		#print 'FIN CARGAR CUBOS: '

	#@QtCore.Slot()
	#def llamarBIK_1(self):
		## LLamar al BIK y pasarle el vector POSE:
		#print 'Preparando vector pose 6D'

		#part = "HEAD" #Parte del cuerpo dle robot que se movera.

		#import RoboCompBodyInverseKinematics
		#pose6D = RoboCompBodyInverseKinematics.Pose6D() #target al que se movera
		#pose6D.x = posicionCubos[0][0]
		#pose6D.y = posicionCubos[0][1]
		#pose6D.z = posicionCubos[0][2]
		#pose6D.rx =  posicionCubos[0][3]
		#pose6D.ry =  posicionCubos[0][4]
		#pose6D.rz =  posicionCubos[0][5]
		#print 'Llamando a BIK con pose6D: ',pose6D

		#weights = RoboCompBodyInverseKinematics.WeightVector() #vector de pesos
		#weights.x = 1
		#weights.y = 1
		#weights.z = 1
		#weights.rx = 0
		#weights.ry = 0
		#weights.rz = 0

		#radius = 150 #radio

		#try:
			#self.bodyinversekinematics_proxy.setTargetPose6D(part,pose6D, weights, radius)
		#except:
			#print "Error en llamarBIK_1"

	#@QtCore.Slot()
	#def llamarBIK_2(self):
		## LLamar al BIK y pasarle el vector POSE:
		#print 'Preparando vector pose 6D'

		#part = "HEAD" #Parte del cuerpo dle robot que se movera.

		#import RoboCompBodyInverseKinematics
		#pose6D = RoboCompBodyInverseKinematics.Pose6D() #target al que se movera
		#pose6D.x = posicionCubos[1][0]
		#pose6D.y = posicionCubos[1][1]
		#pose6D.z = posicionCubos[1][2]
		#pose6D.rx =  posicionCubos[1][3]
		#pose6D.ry =  posicionCubos[1][4]
		#pose6D.rz =  posicionCubos[1][5]
		#print 'Llamando a BIK con pose6D: ',pose6D

		#weights = RoboCompBodyInverseKinematics.WeightVector() #vector de pesos
		#weights.x = 1
		#weights.y = 1
		#weights.z = 1
		#weights.rx = 0
		#weights.ry = 0
		#weights.rz = 0

		#radius = 250 #radio

		#try:
			#self.bodyinversekinematics_proxy.setTargetPose6D(part,pose6D, weights, radius)
		#except:
			#print "Error en llamarBIK_2"

	#@QtCore.Slot()
	#def llamarBIK_3(self):
		## LLamar al BIK y pasarle el vector POSE:
		#print 'Preparando vector pose 6D'

		#part = "HEAD" #Parte del cuerpo dle robot que se movera.

		#import RoboCompBodyInverseKinematics
		#pose6D = RoboCompBodyInverseKinematics.Pose6D() #target al que se movera
		#pose6D.x = posicionCubos[2][0]
		#pose6D.y = posicionCubos[2][1]
		#pose6D.z = posicionCubos[2][2]
		#pose6D.rx =  posicionCubos[2][3]
		#pose6D.ry =  posicionCubos[2][4]
		#pose6D.rz =  posicionCubos[2][5]
		#print 'Llamando a BIK con pose6D: ',pose6D

		#weights = RoboCompBodyInverseKinematics.WeightVector() #vector de pesos
		#weights.x = 1
		#weights.y = 1
		#weights.z = 1
		#weights.rx = 0
		#weights.ry = 0
		#weights.rz = 0

		#radius = 250 #radio

		#try:
			#self.bodyinversekinematics_proxy.setTargetPose6D(part,pose6D, weights, radius)
		#except:
			#print "Error en llamarBIK_3"
