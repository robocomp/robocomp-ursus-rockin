faulhaber=`ls -lah /dev/faulhaber | awk '{print $11}'`
echo $faulhaber
#sed 's/Faulhaber.Device*/Faulhaber.Device=$dunker/g' en /home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/dynamixel.conf

sed "s/Faulhaber.Device.*/Faulhaber.Device=$faulhaber/g" /home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/faulhaber.conf > temp.conf
rm /home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/faulhaber.conf 
mv temp.conf /home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/faulhaber.conf 


dunker=`ls -lah /dev/dunker | awk '{print $11}'`
echo $dunker
#sed 's/Faulhaber.Device*/Faulhaber.Device=$dunker/g' en /home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/dynamixel.conf

sed "s/dunkermotoren.Device.*/dunkermotoren.Device=$dunker/g" /home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/dunker.conf > temp.conf
rm /home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/dunker.conf 
mv temp.conf /home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/dunker.conf 


dynamixel=`ls -lah /dev/dynamixel | awk '{print $11}'`
echo $dynamixel
#sed 's/Faulhaber.Device*/Faulhaber.Device=$dunker/g' en /home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/dynamixel.conf

sed "s/Dynamixel.Device.*/Dynamixel.Device=$dynamixel/g" /home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/dynamixel.conf > temp.conf
rm /home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/dynamixel.conf 
mv temp.conf /home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/dynamixel.conf 

