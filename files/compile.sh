###
### Update repositories
###
# AGM
cd /home/robocomp/AGM
git pull
# robocomp
cd /home/robocomp/robocomp
git pull
# robocomp-robolab
cd /home/robocomp/robocomp/components/robocomp-robolab
git pull
# robocomp-ursus
cd /home/robocomp/robocomp/components/robocomp-ursus
git pull
# robocomp-ursus-rockin
cd /home/robocomp/robocomp/components/robocomp-ursus-rockin
git pull

###
### RoboComp
###
cd /home/robocomp/robocomp/build
make
sudo make install

###
### AGM
###
cd /home/robocomp/AGM
make
sudo make install


###
### COMPONENTS
###

# bik
cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/components/bikComp/
make -j5

# ursuscommonjoint
cd /home/robocomp/robocomp/components/robocomp-ursus/components/ursusCommonJoint/
make -j5

# joystickcomp
cd /home/robocomp/robocomp/components/robocomp-robolab/components/joystickOmniComp/
make -j5

# laserRGBD
cd /home/robocomp/robocomp/components/robocomp-robolab/experimental/laserRGBDComp/
make -j5

# trajectory
cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/components/trajectoryrobot2d/
make -j5

# navigationAgent
cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/components/navigationAgent/
make -j5

# graspingAgent
cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/components/graspingAgent/
make -j5

# objectAgent
cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/components/objectAgent/
make -j5

# apriltags
cd /home/robocomp/robocomp/components/robocomp-robolab/components/apriltagsComp/
make -j5




