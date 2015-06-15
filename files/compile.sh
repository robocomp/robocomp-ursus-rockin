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
if [ $? -ne 0 ]; then
	echo "error compiling robocomp"
	exit
fi
echo "make install robocomp"
sudo make install
if [ $? -ne 0 ]; then
	echo "error installing robocomp"
	exit
fi

###
### AGM
###
echo "make agm"
cd /home/robocomp/AGM
make -j4
if [ $? -ne 0 ]; then
	echo "error compiling agm"
	exit
fi
echo "make install agm"
sudo make install
if [ $? -ne 0 ]; then
	echo "error installing robocomp"
	exit
fi


###
### COMPONENTS
###

# bik
echo "make bik"
cd /home/robocomp/robocomp/components/robocomp-ursus/components/bikComp/
cmake .
make -j5
if [ $? -ne 0 ]; then
	echo "error compiling bik"
	exit
fi

# bik visual
echo "make bik visual"
cd /home/robocomp/robocomp/components/robocomp-ursus/components/visualbik/
cmake .
make -j5
if [ $? -ne 0 ]; then
	echo "error compiling visualbik"
	exit
fi

# ursuscommonjoint
echo "make ursuscommonjoint"
cd /home/robocomp/robocomp/components/robocomp-ursus/components/ursusCommonJoint/
cmake .
make -j5
if [ $? -ne 0 ]; then
	echo "error compiling ursuscommonjoint"
	exit
fi

# joystickcomp
echo "make joystickcomp"
cd /home/robocomp/robocomp/components/robocomp-robolab/components/joystickOmniComp/
cmake .
make -j5
if [ $? -ne 0 ]; then
	echo "error compiling joystickOmni"
	exit
fi

# laserRGBD
echo "make laserRGBD"
cd /home/robocomp/robocomp/components/robocomp-robolab/experimental/laserRGBDComp/
cmake .
make -j5
if [ $? -ne 0 ]; then
	echo "error compiling laserrgbd"
	exit
fi

# trajectory
#echo "make trajectory"
#cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/components/trajectoryrobot2d/
#cmake .
#make -j5
#if [ $? -ne 0 ]; then
#	echo "error compiling trajectory"
#	exit
#fi

# navigationAgent
echo "make navigation agent"
cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/components/navigationAgent/
cmake .
make -j5
if [ $? -ne 0 ]; then
	echo "error compiling navigation agent"
	exit
fi

# graspingAgent
echo "make grasping agent"
cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/components/graspingAgent/
cmake .
make -j5
if [ $? -ne 0 ]; then
	echo "error compiling grasping agent"
	exit
fi

# objectAgent
echo "make object agent"
cd /home/robocomp/robocomp/components/robocomp-ursus-rockin/components/objectAgent/
cmake .
make -j5
if [ $? -ne 0 ]; then
	echo "error compiling object agent"
	exit
fi

# apriltags
echo "make apriltags"
cd /home/robocomp/robocomp/components/robocomp-robolab/components/apriltagsComp/
cmake .
make -j5
if [ $? -ne 0 ]; then
	echo "error compiling apriltags"
	exit
fi

# apriltags
echo "make camara"
cd /home/robocomp/robocomp/components/robocomp-robolab/components/cameraV4lComp/
cmake .
make -j5
if [ $? -ne 0 ]; then
	echo "error compiling cameraV4l"
	exit
fi


# base
echo "make baseursus"
cd /home/robocomp/robocomp/components/robocomp-ursus/components/baseursus/
cmake .
make -j5
if [ $? -ne 0 ]; then
	echo "error compiling baseursus"
	exit
fi






