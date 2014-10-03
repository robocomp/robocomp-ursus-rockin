#!/usr/bin/env python
# -*- coding: utf-8 -*-


import sys, traceback, Ice, threading, time, os
import IceStorm

# Ctrl+c handling
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

# Qt interface
from PySide.QtCore import *
from PySide.QtGui import *
from PySide.QtSvg import *

# Check that RoboComp has been correctly detected
ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	pass
if len(ROBOCOMP)<1:
	print 'ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


Ice.loadSlice("-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/ASRPublish.ice")
import RoboCompASRPublish
Ice.loadSlice("-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/ASRCommand.ice")
import RoboCompASRCommand
Ice.loadSlice("-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/ASRComprehension.ice")
import RoboCompASRComprehension


class MainClass(object):
	def __init__(self, commandTopic):
		print 'Esta clase podria ser la clase principal del programa'
		self.commandTopic = commandTopic
	def newText(self, text, current=None):
		print 'Nos ha llegado', text
		command = RoboCompASRCommand.Command()
		partes = text.split()
		if len(partes) > 0:
			command.action = partes[0]
			if len(partes) > 1:
				command.complements = partes[1:]
				print 'Action', command.action, '(', command.complements,')'
			else:
				print 'Action', command.action
			self.commandTopic.newCommand(command)
		else:
			print 'Comando vacio?'
	def mode(self, text):
		print 'Nos llega por la interfaz ASRComprehension', text

class ASRPublishTopicI (RoboCompASRPublish.ASRPublish):
	def __init__(self, _handler):
		self.handler = _handler
	def newText(self, text, current=None):
		self.handler.newText(text)

class ASRComprehensionI (RoboCompASRComprehension.ASRComprehension):
	def __init__(self, _handler):
		self.handler = _handler
	def mode(self, text, current=None):
		self.handler.mode(text)

class Server (Ice.Application):
	def run (self, argv):
		status = 0
		try:
			# Proxy to publish ASRCommand
			proxy = self.communicator().getProperties().getProperty("IceStormProxy")
			obj = self.communicator().stringToProxy(proxy)
			topicManager = IceStorm.TopicManagerPrx.checkedCast(obj)
			try:
				topic = False
				topic = topicManager.retrieve("ASRCommand")
			except:
				pass
			while not topic:
				try:
					topic = topicManager.retrieve("ASRCommand")
				except IceStorm.NoSuchTopic:
					try:
						topic = topicManager.create("ASRCommand")
					except:
						print 'Another client created the ASRCommand topic... ok'
			pub = topic.getPublisher().ice_oneway()
			commandTopic = RoboCompASRCommand.ASRCommandPrx.uncheckedCast(pub)

			mainObject = MainClass(commandTopic)


			# Subscribe to ASRPublishTopic
			proxy = self.communicator().getProperties().getProperty( "IceStormProxy")
			topicManager = IceStorm.TopicManagerPrx.checkedCast(self.communicator().stringToProxy(proxy))
			adapterT = self.communicator().createObjectAdapter("ASRPublishTopic")
			asrTopic = ASRPublishTopicI(mainObject)
			proxyT = adapterT.addWithUUID(asrTopic).ice_oneway()
			ASRPublishTopic_subscription = False
			while not ASRPublishTopic_subscription:
				try:
					topic = topicManager.retrieve("ASRPublishTopic")
					qos = {}
					topic.subscribeAndGetPublisher(qos, proxyT)
					adapterT.activate()
					ASRPublishTopic_subscription = True
				except IceStorm.NoSuchTopic:
					print "Error! No topic found! Sleeping for a while..."
					time.sleep(1)
			print 'ASRPublishTopic subscription ok'


			# Implement ASRComprehension
			asrcomprehensionI = ASRComprehensionI(mainObject)
			adapterASRComprehension = self.communicator().createObjectAdapter('ASRComprehension')
			adapterASRComprehension.add(asrcomprehensionI, self.communicator().stringToIdentity('asrcomprehension'))
			adapterASRComprehension.activate()



			self.communicator().waitForShutdown()
		except:
			traceback.print_exc()
			status = 1

		if self.communicator():
			try:
				self.communicator().destroy()
			except:
				traceback.print_exc()
				status = 1

Server( ).main(sys.argv)
