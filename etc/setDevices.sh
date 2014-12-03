dunker=`ls -lah /dev/dunker | awk '{print $11}'`
echo $dunker
sed POLLAS   cambioi "Faulhaber.Device FINDELINEA" por "Faulhaber.Device=$dunker" en /home/robocomp/components/robocomp-ursus-rockin/etc/dynamixel.conf





