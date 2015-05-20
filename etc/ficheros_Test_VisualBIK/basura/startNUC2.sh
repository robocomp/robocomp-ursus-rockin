######################################################################
###### PARA LEVANTAR LO QUE HAY EN EL NUC2
######		PRIMESENSE
######		DUNKER
######################################################################

# URSUS DUNKER
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
#qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'sshpass -p opticalflow ssh robonuc2@nuc2'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd robocomp/components/robocomp-robolab/components/dunkermotorenComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'killall -9 dunkermotorenComp'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake . && make && bin/dunkermotorenComp --Ice.Config=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/dunker.conf'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess '2_Dunker Ursus'
sleep 1