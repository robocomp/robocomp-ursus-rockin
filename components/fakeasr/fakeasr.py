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


class MainClass(QWidget):
	def __init__(self, asrTopicPrx):
		QWidget.__init__(self)
		self.asrTopicPrx = asrTopicPrx
		self.timer = QTimer()
		self.timer.start(1000)
		self.setupUi()
		self.connect(self.sendButton, SIGNAL('clicked()'), self.sendSlot)
		self.show()
	def setupUi(self):
		self.resize(687, 45)
		self.horizontalLayout = QHBoxLayout(self)
		self.horizontalLayout.setObjectName("horizontalLayout")
		self.lineEdit = QLineEdit(self)
		self.lineEdit.setObjectName("lineEdit")
		self.horizontalLayout.addWidget(self.lineEdit)
		self.sendButton = QPushButton(self)
		self.sendButton.setObjectName("sendButton")
		self.horizontalLayout.addWidget(self.sendButton)

		self.retranslateUi(self)
		QMetaObject.connectSlotsByName(self)

	def retranslateUi(self, Form):
		Form.setWindowTitle(QApplication.translate("Form", "Form", None, QApplication.UnicodeUTF8))
		self.sendButton.setText(QApplication.translate("Form", "send", None, QApplication.UnicodeUTF8))

	def sendSlot(self):
		self.asrTopicPrx.newText(self.lineEdit.text())


class Server (Ice.Application):
	def run (self, argv):
		status = 0
		try:
			# Proxy to publish ASR
			proxy = self.communicator().getProperties().getProperty("IceStormProxy")
			obj = self.communicator().stringToProxy(proxy)
			topicManager = IceStorm.TopicManagerPrx.checkedCast(obj)
			try:
				topic = False
				topic = topicManager.retrieve("ASRPublishTopic")
			except:
				pass
			while not topic:
				try:
					topic = topicManager.retrieve("ASRPublishTopic")
				except IceStorm.NoSuchTopic:
					try:
						topic = topicManager.create("ASRPublishTopic")
					except:
						print 'Another client created the ASRPublishTopic topic... ok'
			pub = topic.getPublisher().ice_oneway()
			asrTopic = RoboCompASRPublish.ASRPublishPrx.uncheckedCast(pub)


			app = QApplication(sys.argv)
			aaa = MainClass(asrTopic)

			app.exec_()
			#self.communicator().waitForShutdown()
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
