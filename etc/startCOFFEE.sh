# RCIS
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/files/makeMeCoffee'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rcis simulation.xml'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'rcis'

# Speech
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-robolab/components/speechComp/'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'sh startSpeech.sh'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'speech'


# Ice Storm
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/etc'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'icebox --Ice.Config=config.icebox'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'storm'
sleep 3


# joystickComp
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-robolab/components/joystickComp/bin'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand './joystickComp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/joystick.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'joystick'
sleep 1

#trajectortrobot
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/trajectoryrobot2d/bin'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand './trajectoryrobot2dcomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/trajectoryrobot2dCoffee.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'trajectory'
sleep 1

# navigationAgent
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/components/navigationAgent'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/navigationcomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/navigationAgentCOFFEE.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'navigationAgent'

# Mission
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus/components/missionAgent'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/missionagent --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/missionCOFFEE.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'mission'



# #trajectorytester
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
# sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/trajectorytester/bin'
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand './trajectorytestercomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/trajectorytester.conf'
# qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'tester'
# sleep 1
#

#
# # AprilTags
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
# sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-robolab/components/apriltagsComp'
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'bin/apriltagscomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/apriltags.conf'
# qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'april'
# sleep 1
#
# # ikComp
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
# sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/inversekinematicsComp'
# qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/lokiarmcomp  --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/inversekinematics.conf'
# qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'ikComp'



# AGM Executive
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/AGM/tools/AGMExecutive_robocomp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'python AGMExecutive_robocomp.py --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/executiveCOFFEE.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'executive'
sleep 1

