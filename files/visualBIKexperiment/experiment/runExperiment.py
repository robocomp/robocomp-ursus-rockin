import sys, os, Ice, traceback, math, random, copy
import numpy as np
import commands
import time
import subprocess
from PySide import *
from PyQt4 import QtGui
from PyQt4.QtGui import QApplication, QDialog
try:
	from ui_MainWindow import *
except:
	print "Can't import UI file. Did you run 'make'? or pyuic4 mainUI.ui -o mainUI.py?"
	sys.exit(-1)
	
from auxiliarExperiment import *

#------------------------------------------------------------------------------------------------------
# Este es el fichero principal que levanta la interfaz de usuario y llama a la clase encargada de 
# llevar a cabo todas las opciones de esa interfaz.
#------------------------------------------------------------------------------------------------------
#------------------------------------------------------#
#                   PROGRAMA PRINCIPAL                 #
#------------------------------------------------------#
if __name__ == '__main__':
	print "\n\nEMPEZAMOS EL PROGRAMA\n\n"	
	app = QtGui.QApplication(sys.argv)
	
	params = copy.deepcopy(sys.argv)
	if len(params) > 1:
		if not params[1].startswith('--Ice.Config='):
			params[1] = '--Ice.Config=' + params[1]
	elif len(params) == 0:
		params.append('--Ice.Config=config')
	
	aux = Auxiliar(params)
	app.exec_()
	
	print "\n\nFIN DEL PROGRAMA\n\n"
# FIN DEL MAIN