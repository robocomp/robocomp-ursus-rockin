# Ejecutamos el COMPILE.SH:
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
#sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd robocomp/components/robocomp-ursus-rockin/files'
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'sh compile.sh'
#qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'Compile.sh'
#sleep 2

# Levantamos el RCIS sin errores:
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd robocomp/components/robocomp-ursus-rockin/etc/ficheros_Test_VisualBIK/innerModelsWorlds'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 rcis'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rcis pruebaRockin.xml'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'Rcis'
sleep 2

# Levantamos el RCREMOTESERVER
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd robocomp/components/robocomp-ursus/etc'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rcremoteserver'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'Rcremoteserver'
sleep 3

# Levantamos el RCMANAGER
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd robocomp/components/robocomp-ursus-rockin/etc/ficheros_Test_VisualBIK/'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'rcmanager ursusmanagerSim.xml'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'Rcmanager'
sleep 3  


