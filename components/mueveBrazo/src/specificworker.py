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
		


	def setParams(self, params):
		#try:
		#	par = params["InnerModelPath"]
		#	innermodel_path=par.value
		#	innermodel = InnerModel(innermodel_path)
		#except:
		#	tracebak.print_exc()
		#	print "Error reading config params"
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

		
		
	


