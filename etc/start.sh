## Script for Ursus-Rockin

#RCIS
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/components/robocomp-ursus-rockin/files/RoCKIn@home/world'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rcis rockinSimple.xml'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'rcis'
sleep 1
#joystickComp
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-robolab/components/joystickComp/bin'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand './joystickComp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/joystick.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'joystick'
sleep 1
#trajectorytester
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/trajectorytester/bin'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand './trajectorytestercomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/trajectorytester.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'tester'
sleep 1
#trajectortrobot
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/trajectoryrobot2d/bin'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand './trajectoryrobot2dcomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/trajectoryrobot2d.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'traj'
sleep 1
