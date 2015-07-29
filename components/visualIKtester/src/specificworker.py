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
		
		self.ui.txbox.setValue(100)
		self.ui.tybox.setValue(850)
		self.ui.tzbox.setValue(400)
		self.ui.rxbox.setValue(0)
		self.ui.rybox.setValue(-1.56)
		self.ui.rzbox.setValue(-3.1415)
		self.ui.weightbutton.setChecked(True)
		
		self.ui.homebutton.clicked.connect(self.goHome)
		self.ui.homebutton_2.clicked.connect(self.goHome)
		self.ui.stopbutton.clicked.connect(self.stop)
		self.ui.stopbutton_2.clicked.connect(self.stop)
		self.ui.sendbutton.clicked.connect(self.sendPose6D)
		self.ui.alignaxisbutton.clicked.connect(self.sendPoseAlignAxis)
		#self.ui.advanceaxisbutton.clicked.connect(self.sendPoseAdvanceAxis)
		self.mapa = {'rightShoulder1':-0.5, 'rightShoulder2':-0.70, 'rightShoulder3':.50 , 'rightElbow':1.30 , 'rightForeArm':-.689, 'head_yaw_joint':0.30, 'head_pitch_joint':0.20}
		
		#self.prueba10puntos()
		
		#while True:
			#if os.path.exists("/home/robocomp/robocomp/components/robocomp-ursus/components/visualik/data.txt") == False:
				#print "El fichero no existe"
				#sys.exit(-1)
				
			#if self.inversekinematics_proxy.getPartState("RIGHTARM") == True:
				#sys.exit(0)

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
		#print 'SpecificWorker.compute...'
		#try:
		#	differentialrobot_proxy.setSpeed(100, 0)
		#except Ice.Exception, e:
		#	traceback.print_exc()
		#	print e
		return True
	
	@QtCore.Slot()
	def goHome(self):
		print "GO HOME"
		import RoboCompInverseKinematics
		try:
			self.inversekinematics_proxy.goHome("RIGHTARM");
		except RoboCompInverseKinematics.IKException, e:
			print "Exception in tester (GO HOME)): ", e
			
	@QtCore.Slot()
	def stop(self):
		print "STOP"
		import RoboCompInverseKinematics
		try:
			self.inversekinematics_proxy.stop("RIGHTARM");
		except RoboCompInverseKinematics.IKException, e:
			print "Exception in tester (STOP): ", e
	
	@QtCore.Slot()
	def sendPose6D(self):
		print "SEND POSE 6D"
		import RoboCompInverseKinematics
		pose6D = RoboCompInverseKinematics.Pose6D() #target al que se movera
		pose6D.x  = float(self.ui.txbox.value())
		pose6D.y  = float(self.ui.tybox.value())
		pose6D.z  = float(self.ui.tzbox.value())
		pose6D.rx = float(self.ui.rxbox.value())
		pose6D.ry = float(self.ui.rybox.value())
		pose6D.rz = float(self.ui.rzbox.value())
		print '---> ',pose6D
		weights = RoboCompInverseKinematics.WeightVector() #vector de pesos
		weights.x = 1
		weights.y = 1
		weights.z = 1
		if self.ui.weightbutton.isChecked() == True:
			weights.rx = 0.1
			weights.ry = 0.1
			weights.rz = 0.1
		else:
			weights.rx = 0
			weights.ry = 0
			weights.rz = 0
		try:
			axis = RoboCompInverseKinematics.Axis() #vector de pesos
			axis.x = 0
			axis.y = 0
			axis.z = 1
			part = "HEAD"
			self.inversekinematics_proxy.setTargetAlignaxis(part, pose6D, axis)
			
			part = "RIGHTARM"
			identificador = self.inversekinematics_proxy.setTargetPose6D(part,pose6D, weights)
			print 'Mirando estado'
			state = RoboCompInverseKinematics.TargetState()
			state = self.inversekinematics_proxy.getTargetState("RIGHTARM", identificador)
			while state.finish!=True:
				state = self.inversekinematics_proxy.getTargetState("RIGHTARM", identificador)
			print 'Moviemdo motores!!'
#			for motor in state.motors:
#				print motor
#				goal = MotorGoalPosition()
#				goal.position = motor.angle
#				goal.name = motor.name
#				goal.maxSpeed = 0.5
#				try:
#					self.jointmotor_proxy.setPosition(goal)
#				except CollisionException:
#					print "Error en arriba_R: ",CollisionException
		except RoboCompInverseKinematics.IKException, e:
			print "Expection in tester (sendPose): ", e
				
	@QtCore.Slot()
	def sendPoseAlignAxis(self):
		#  setTargetAlignaxis(const string &bodyPart, const Pose6D &target, const Axis &ax)
		print "SEND POSE ALIGN AXIS"
		import RoboCompInverseKinematics
		pose6D = RoboCompInverseKinematics.Pose6D() #target al que se movera
		pose6D.x  = float(self.ui.txbox.value())
		pose6D.y  = float(self.ui.tybox.value())
		pose6D.z  = float(self.ui.tzbox.value())
		pose6D.rx = float(self.ui.rxbox.value())
		pose6D.ry = float(self.ui.rybox.value())
		pose6D.rz = float(self.ui.rzbox.value())
		print '---> ',pose6D
		axis = RoboCompInverseKinematics.Axis() #vector de pesos
		axis.x = 0
		axis.y = 0
		axis.z = 1
		
		try:
			part = "HEAD"
			self.inversekinematics_proxy.setTargetAlignaxis(part, pose6D, axis)
		except RoboCompInverseKinematics.IKException, e:
			print "Expection in tester (sendPose): ", e
			
	
	def prueba10puntos(self):
		import RoboCompInverseKinematics
		for i in range(0, 10):
			pose6D = RoboCompInverseKinematics.Pose6D() #target al que se movera
			pose6D.x  = random.randint(140, 300)
			pose6D.y  = random.randint(780, 850)
			pose6D.z  = random.randint(370, 400)
			pose6D.rx = 0
			pose6D.ry = 0
			pose6D.rz = 3.1416
			print 'i: ',i," pose: ", pose6D
			#print 'Llamando a BIK con pose6D: ',pose6D
			#self.ui.label_13.setText(self.ui.label_13.getText()+'\n('+str(pose6D.x)+', '+str(pose6D.y)+', '+str(pose6D.z)+'), ['+str(pose6D.rx)+', '+str(pose6D.ry)+', '+str(pose6D.rz)+']')

			weights = RoboCompInverseKinematics.WeightVector() #vector de pesos
			weights.x = 1
			weights.y = 1
			weights.z = 1
			weights.rx = 0.1
			weights.ry = 0.1
			weights.rz = 0.1
			#try:
			#part = "HEAD"
			#self.inversekinematics_proxy.setTargetAlignaxis(part, pose6D, axis)
			part = "RIGHTARM"
			self.inversekinematics_proxy.setTargetPose6D(part,pose6D, weights)


