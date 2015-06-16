## Mi Script para levantar URSUS para mañana
# RECUERDA ESTAR CONECTADA A URSUS LOCAL

##################################################################
##################################### PRIMERO ME CONECTO CON NUC1:
##################################################################
# URSUS BASE
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'sshpass -p opticalflow ssh robolab@nuc1'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd robocomp/components/robocomp-ursus-rockin/components/baseursus/'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 baseursuscomp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/baseursuscomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/base.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess '1_Base Ursus'
sleep 1

# URSUS JOYSTICK
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'sshpass -p opticalflow ssh robolab@nuc1'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd robocomp/components/robocomp-robolab/components/joystickOmniComp/'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 joystickOmniComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/joystickOmniComp --Ice.Config=config '
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess '1_Joystick Base Ursus'
sleep 1

# URSUS FAULHABER
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'sshpass -p opticalflow ssh robolab@nuc1'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /robocomp/components/robocomp-ursus/components/faulhaberComp/'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 faulhaberComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake && make && bin/faulhaberComp --Ice.Config=../../../robocomp-ursus-rockin/etc/faulhaber.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess '1_Faulhaber Ursus'
sleep 1

# URSUS DYNAMIXEL
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'sshpass -p opticalflow ssh robolab@nuc1'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /robocomp/components/robocomp-robolab/components/dynamixelComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 dynamixelComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/dynamixelComp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/dynamixel.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess '1_Dynamixel Ursus'
sleep 1

# URSUS COMMONJOINT
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'sshpass -p opticalflow ssh robolab@nuc1'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /robocomp/components/robocomp-ursus/components/ursusCommonJoint'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 ursuscommonjointcomp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make &&  bin/ursuscommonjointcomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/ursusCommon.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess '1_CommonJoint Ursus'
sleep 1

##################################################################
##################################### DESPUES ME CONECTO CON NUC2:
##################################################################
# URSUS DUNKER
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'sshpass -p opticalflow ssh robonuc2@nuc2'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd robocomp/components/robocomp-robolab/components/dunkermotorenComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 dunkermotorenComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/dunkermotorenComp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/dunker.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess '2_Dunker Ursus'
sleep 1

##################################################################
##################################### LEVANTO COMPONENTES EN LOCAL:
##################################################################
# ICE STORM
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd robocomp/components/robocomp-ursus-rockin/etc/pablo'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 icebox'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'icebox --Ice.Config=config.icebox'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess '0_Storm'
sleep 1

# RCIS
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd robocomp/components/robocomp-ursus-rockin/etc/'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 rcis'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rcis im.xml'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess '0_Rcis'
sleep 1

# MONITOR
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd robocomp/tools/rcmonitor/'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'python rcmonitor.py examples/jointMotorSimple.rcm -p 20000'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess '0_Monitor'
sleep 1

#FALCON
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd robocomp/components/robocomp-robolab//components/joystickpublishfalconComp/'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 JoystickPublishFalcon'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/JoystickPublishFalcon --Ice.Config=../../../robocomp-ursus-rockin/etc/mercedes/DEFINITIVO_FALCON.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess '0_Falcon'
sleep 1

#BIK
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd robocomp/components/robocomp-ursus-rockin/components/inversekinematicsComp/'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 bik'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/bik --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/pablo/inversekinematics.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess '0_BIK'
sleep 1