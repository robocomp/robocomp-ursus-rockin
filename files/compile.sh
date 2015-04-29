###
### Update repositories
###
# AGM
echo "update agm"
cd /home/robocomp/AGM
git pull
# robocomp
echo "update robocomp"
cd /home/robocomp/robocomp
git pull
# robocomp-robolab
echo "update robocomp-robolab"
cd /home/robocomp/robocomp/components/robocomp-robolab
git pull
# robocomp-ursus
echo "update robocomp-ursus"
cd /home/robocomp/robocomp/components/robocomp-ursus
git pull
# robocomp-ursus-rockin
echo "update robocomp-ursus-rockin"
cd /home/robocomp/robocomp/components/robocomp-ursus-rockin
git pull

###
### RoboComp
###
echo "make robocomp"
cd /home/robocomp/robocomp/build
make -j4
echo "make install robocomp"
sudo make install

###
### AGM
###
echo "make agm"
cd /home/robocomp/AGM
make -j4
echo "make install agm"
sudo make install


###
### COMPONENTS
###

# bik
echo "make bik"
cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/components/bikComp/
cmake .
make -j5

# ursuscommonjoint
echo "make ursuscommonjoint"
cd /home/robocomp/robocomp/components/robocomp-ursus/components/ursusCommonJoint/
cmake .
make -j5

# joystickcomp
echo "make joystickcomp"
cd /home/robocomp/robocomp/components/robocomp-robolab/components/joystickOmniComp/
cmake .
make -j5

# laserRGBD
echo "make laserRGBD"
cd /home/robocomp/robocomp/components/robocomp-robolab/experimental/laserRGBDComp/
cmake .
make -j5

# trajectory
echo "make trajectory"
cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/components/trajectoryrobot2d/
cmake .
make -j5

# navigationAgent
echo "make navigation agent"
cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/components/navigationAgent/
cmake .
make -j5

# graspingAgent
echo "make grasping agent"
cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/components/graspingAgent/
cmake .
make -j5

# objectAgent
echo "make object agent"
cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/components/objectAgent/
cmake .
make -j5

# apriltags
echo "make apriltags"
cd /home/robocomp/robocomp/components/robocomp-robolab/components/apriltagsComp/
cmake .
make -j5

# apriltags
echo "make camara"
cd /home/robocomp/robocomp/components/robocomp-robolab/components/cameraV4lComp/
cmake .
make -j5




