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

import sys, os, Ice, traceback, math
import numpy as np

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
Ice.loadSlice(preStr+"OmniRobot.ice")
from RoboCompOmniRobot import *
Ice.loadSlice(preStr+"TrajectoryRobot2D.ice")
from RoboCompTrajectoryRobot2D import *
Ice.loadSlice(preStr+"Laser.ice")
from RoboCompLaser import *


from trajectoryrobot2dI import *

def saturate_minabs_BothSigns(value, minabs, top):
	if value > top:
		return top
	elif value < -top:
		return -top
	elif abs(value) < minabs:
		return 0.
	return value

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.hide()

		self.state = NavState()
		self.updateStatePose()
		self.state.advV = self.state.rotV = 0;
		self.state.distanceToTarget = 0
		self.state.elapsedTime = 0
		self.state.estimatedTime = 0
		self.state.planningTime = 0
		self.state.state = "IDLE"

		self.updateStatePose()
		target = TargetPose()
		target.doRotation = False
		target.x  = self.state.x
		target.y  = 0
		target.z  = self.state.z
		target.rx = 0
		target.ry = self.state.ry
		target.rz = 0
		self.goReferenced(target, 0, 0, 0)
		self.stop()


		self.Period = 100
		self.timer.start(self.Period)
		self.timer.timeout.connect(self.compute)

	def normalize_s_pi(self, angle):
		r = float(angle) - (math.pi*2.) * math.floor( angle / (math.pi*2.) )
		if r > math.pi:
			r -= 2.*math.pi
		return r

	def updateStatePose(self):
		self.state.x, self.state.z, self.state.ry = self.omnirobot_proxy.getBasePose()
		print 'POSE', self.state.x, self.state.z, self.state.ry

	def setParams(self, params):
		return True

	def getError(self, current, target):
		errAlpha = self.normalize_s_pi(self.normalize_s_pi(target.ry)-self.normalize_s_pi(current.ry))
		return target.x-current.x, target.z-current.z, errAlpha

	@QtCore.Slot()
	def compute(self):
		l = QtCore.QMutexLocker(self.mutex)

		if self.state.state == 'IDLE':
			pass
		elif self.state.state == 'EXECUTING':
			try:
				self.updateStatePose()
				errX, errZ, errAlpha = self.getError(self.state, self.target)
				# Compute initial relative error
				self.relErrX = errX*math.cos(self.state.ry) - errZ*math.sin(self.state.ry)
				self.relErrZ = errX*math.sin(self.state.ry) + errZ*math.cos(self.state.ry)
				# Now we take into account the target reference
				self.relErrX -= self.xRef
				self.relErrZ -= self.zRef
				# Final relative coordinates of the target
				print 'command', self.relErrX, self.relErrZ
				
				proceed = True
				command = np.array([self.relErrX, self.relErrZ])
				#print 'norm', np.linalg.norm(command), 'threshold', self.threshold
				if np.linalg.norm(command)<=self.threshold and abs(errAlpha) < 0.08:
					print 'stop by threshold'
					proceed = False
				if proceed:
					maxspeed = 300.
					if np.linalg.norm(command)<0.1:
						command = np.array([0,0])
					else:
						speed = np.linalg.norm(command)
						if speed > maxspeed: speed = maxspeed
						command = command / (np.linalg.norm(command)/speed)
					commandAlpha = saturate_minabs_BothSigns(errAlpha, 0.05, 0.3)
					self.omnirobot_proxy.setSpeedBase(command[0], command[1], commandAlpha)
				else:
					print '<Now IDLE'
					self.stop()
					self.state.state = 'IDLE'
					print 'Now IDLE>'

			except Ice.Exception, e:
				traceback.print_exc()
				print e
		else:
			print 'Internal error: unknown state:', self.state.state
			return False
		return True


	#
	# go
	def go(self, target):
		print target.x, target.z
		return self.goReferenced(target, 0, 0, 0)


	# goReferenced
	#
	def goReferenced(self, target, xRef, zRef, threshold):
		l = QtCore.QMutexLocker(self.mutex)
		self.state.state = "EXECUTING"
		self.target = target
		self.xRef = float(xRef)
		self.zRef = float(zRef)
		self.threshold = threshold
		if self.threshold < 20.:
			self.threshold = 20.
		
		self.compute()
		dist = math.sqrt(self.relErrX**2. + self.relErrZ**2.)
		return dist


	#
	# getState
	def getState(self):
		l = QtCore.QMutexLocker(self.mutex)
		return self.state

	#
	# goBackwards
	def goBackwards(self, target):
		return self.changeTarget(target)

	#
	# stop
	def stop(self):
		l = QtCore.QMutexLocker(self.mutex)
		self.omnirobot_proxy.setSpeedBase(0,0,0)


	#
	# changeTarget
	def changeTarget(self, target):
		return self.goReferenced(target, 0, 0, 0)





