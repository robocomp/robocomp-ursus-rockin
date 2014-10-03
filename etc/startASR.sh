# Ice Storm
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/etc'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'icebox --Ice.Config=config.icebox'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'storm'
sleep 1

# Speech
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-robolab/components/speechComp/'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'sh startSpeech.sh'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'speech'

# Fake ASR
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/components/fakeasr'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'python fakeasr.py --Ice.Config=config'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'fakeasr'

# Comprehension
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/components/comprehension'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'python comprehension.py --Ice.Config=config'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'comprehension'


# Dialog
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
sess=`qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId`
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/components/dialogAgent'
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.runCommand 'cmake .; make -j 1 && bin/hricomp --Ice.Config=config'
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle $sess 'dialog'


