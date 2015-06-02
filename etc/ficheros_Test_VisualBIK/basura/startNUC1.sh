######################################################################
###### PARA LEVANTAR LO QUE HAY EN EL NUC1
######		BASE URSUS
######		DYNAMIXEL
######		FAULHABER
######		HUB:
######			PANTALLA
######			JOYSTICK
######			LASER
######################################################################

# URSUS BASE
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'sshpass -p opticalflow ssh robolab@nuc1'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd robocomp/components/robocomp-ursus-rockin/components/baseursus/'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 baseursuscomp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/baseursuscomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/base.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess '1_Base Ursus'
sleep 1

# URSUS JOYSTICK
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'sshpass -p opticalflow ssh robolab@nuc1'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd robocomp/components/robocomp-robolab/components/joystickOmniComp/'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 joystickOmniComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/joystickOmniComp --Ice.Config=config '
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess '1_Joystick Base Ursus'
sleep 1

# URSUS FAULHABER
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'sshpass -p opticalflow ssh robolab@nuc1'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /robocomp/components/robocomp-ursus/components/faulhaberComp/'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 faulhaberComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake && make && bin/faulhaberComp --Ice.Config=../../../robocomp-ursus-rockin/etc/faulhaber.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess '1_Faulhaber Ursus'
sleep 1

# URSUS DYNAMIXEL
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'sshpass -p opticalflow ssh robolab@nuc1'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /robocomp/components/robocomp-robolab/components/dynamixelComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 dynamixelComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/dynamixelComp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/dynamixel.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess '1_Dynamixel Ursus'
sleep 1

# URSUS COMMONJOINT
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'sshpass -p opticalflow ssh robolab@nuc1'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /robocomp/components/robocomp-ursus/components/ursusCommonJoint'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 ursuscommonjointcomp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make &&  bin/ursuscommonjointcomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/ursusCommon.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess '1_CommonJoint Ursus'
sleep 1