# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'src/mainUI.ui'
#
# Created: Sat Mar  7 22:49:16 2015
#      by: pyside-uic 0.2.15 running on PySide 1.2.2
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_guiDlg(object):
    def setupUi(self, guiDlg):
        guiDlg.setObjectName("guiDlg")
        guiDlg.resize(400, 300)

        self.retranslateUi(guiDlg)
        QtCore.QMetaObject.connectSlotsByName(guiDlg)

    def retranslateUi(self, guiDlg):
        guiDlg.setWindowTitle(QtGui.QApplication.translate("guiDlg", "name", None, QtGui.QApplication.UnicodeUTF8))

