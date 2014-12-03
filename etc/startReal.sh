sh setDevices.sh


# Ice Storm
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/etc'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'icebox --Ice.Config=config.icebox'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'storm'

# Speech
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-robolab/components/speechComp/'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand '#sh startSpeech.sh'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'speech'
sleep 1

# opticalflow
# faulhaber
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus/components/faulhaberComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 faulhaberComp' 
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand '#cmake . && make -j1 && ./bin/faulhaberComp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/faulhaber.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'Faulhaber'
sleep 3

# dunker
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-robolab/components/dunkermotorenComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 faulhaberComp' 
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand '#cmake . && make -j1 && ./bin/dunkermotorenComp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/dunker.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'dunker'
sleep 3


# # primesense
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
# sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-robolab/experimental/primeSenseComp'
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 primeSenseComp'
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make -j1 && ./bin/primeSenseComp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-XXXXXXXXXXXX/etc/primeSense.conf'
# qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'primesense'
# 

# dynamixel
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-robolab/components/dynamixelComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 dynamixelComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand '#cmake . && make -j1 && ./bin/dynamixelComp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/dynamixel.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'Dynamixel'
sleep 3

# base
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/components/baseursus'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 baseursus'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand '#cmake . && make -j1 && ./bin/baseursuscomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/base.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'BASE'
sleep 3

# joystickOmni
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-robolab/components/joystickOmniComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 joystickOmniComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand '#cmake . && make -j1 && ./bin/joystickOmniComp --Ice.Config=config'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'JOYSTICK'
sleep 3


# common jointProxy
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus/components/ursusCommonJoint'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 ursuscommonjointcomp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand '#cmake . && make -j1 && ./bin/ursuscommonjointcomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/ursusCommon.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'JointProxy'
sleep 2


# # ikComp
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
# sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus/components/inversekinematicsComp'
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 lokiarmcomp'
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make -j1 && ./bin/lokiarmcomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-XXXXXXXXXXXX/etc/ikComp.conf'
# qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'ikComp'
# sleep 2
# 
# # apriltagsComp
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
# sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-robolab/components/apriltagsComp'
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 apriltagscomp'
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make -j1 && bin/apriltagscomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-XXXXXXXXXXXX/etc/aprilComp.conf'
# qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'aprilComp'
# sleep 2
# 
# # ikAgent
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
# sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus-XXXXXXXXXXXX/components/inversekinematicsAgent'
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 inversekinematicsagentcomp'
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make -j1 && bin/inversekinematicsagentcomp  --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-XXXXXXXXXXXX/etc/ikAgent.conf'
# qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'ikAgent'
# sleep 1
# # apriltagsAgent
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
# sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus-XXXXXXXXXXXX/components/apriltagsAgentComp'
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 apriltagsagentcomp'
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make -j1 && bin/apriltagsagentcomp  --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-XXXXXXXXXXXX/etc/aprilAgent.conf'
# qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'aprilAgent'
# sleep 1
# 
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
# sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus/components/missionAgent'
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 bin/missionagent'
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make -j1 && bin/missionagent  --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-XXXXXXXXXXXX/etc/mission.conf'
# qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'mission'
# sleep 1
# 
# 
# # AGM Executive
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
# sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/AGM/tools/AGMExecutive_robocomp'
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'python AGMExecutive_robocomp.py --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-XXXXXXXXXXXX/etc/executive.conf'
# qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'executive'
# sleep 1
