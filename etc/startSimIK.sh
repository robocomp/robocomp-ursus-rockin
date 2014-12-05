# RCIS
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/files/makeMeCoffee'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rcis simulation.xml'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'rcis'
sleep 4

# Ice Storm
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/etc'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'icebox --Ice.Config=config.icebox'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'storm'
sleep 2

# ursuscommonjoint
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus/components/ursusCommonJoint'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 ursuscommonjointcomp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make -j 1 && bin/ursuscommonjointcomp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/ursusCommonSim.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'commonJoint'
sleep 2

# ikComp
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/inversekinematicsComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 lokiarmcomp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make -j 1 && bin/lokiarmcomp  --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/inversekinematicsCOFFEE.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'ikComp'
sleep 2

# ikTester
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/inversekinematicsComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 lokiarmtestercomp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make -j 1 && bin/lokiarmtestercomp  --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/inversekinematicsCOFFEE.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'ikTest'
sleep 2


