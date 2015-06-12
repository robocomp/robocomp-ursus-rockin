VisualBIK
===============================
##What is Visual Body Inverse Kinematic (VisualBIK)?

In the inversedKinematic component (in robolab-ursus folder) we said that inverse kinematic (IK) is responsible for calculating the angle of each motor (joint) of a kinematic chain, in order to move the end effector from a start point A to an endpoint B. But, what happends if the range of the joints is not correct or there are some calibration errors due to the looseness of the motors? In this case, the internal model can say that the goal position has been reached by the end effector, by in the reality, this is not so.

This visualBIK component aims to avoid that problem. It uses three different positions:

1) The target pose: the goal position in the world system reference. This position will not change until it is resolved.

2) The internal pose of the end effector: This is the position where the robot believes it has its end effector.

3) The visual pose of the end effector: This is the position of the aprilTag (located on the end effector) that the robot's camera sees.

Basically the algorithm compares the aprilTag pose, that the camera sees, with the position of the target, and calculates the errors. These errors are added to a corrected pose. Thus we absorb the calibration error.

##Component that completes inverse kinematics

This component completes the correct behaviour of the inverse kinematics component. Its operation is simple, for now it only works with the right arm of the robot Ursus, and save in a file the results. In the Configuration parameters, we can connect this component with other components through the port 10240. Also, we need to connect this component with others, for example, look the visualBIK/etc/configDefinitivo file:

    BodyInverseKinematics.Endpoints=tcp -p 10240
    CommonBehavior.Endpoints=tcp -p 10000

    AprilTagsTopic=tcp -p 12938  #We need to subscribes to the aprilTags component
    BodyInverseKinematicsProxy = bodyinversekinematics:tcp -h localhost -p 10220
    JointMotorProxy = jointmotor:tcp -p 20000 
    OmniRobotProxy = omnirobot:tcp -p 12238
    InnerModel=/home/robocomp/robocomp/components/robocomp-ursus-rockin/etc/ficheros_Test_VisualBIK/ursus_bik.xml
    
    # This property is used by the clients to connect to IceStorm.
    TopicManager.Proxy=IceStorm/TopicManager:default -p 9999
    
    Ice.Warn.Connections=0
    Ice.Trace.Network=0
    Ice.Trace.Protocol=0
    Ice.ACM.Client=10
    Ice.ACM.Server=10
    
Like you can see, we need to subscribe the visualBIK component to the aprilTags component, in order to take the visual position of the aprilTag. We need the inverseKinematic component too (it is whicht moves the body of the robot) and the robot.xml, with the internal model of the robot. 





















1) the robot whicht will calculate the visualBIK, for example, Ursus. You must put the path of the robot.xml file

    InnerModel=/home/robocomp/robocomp/components/robocomp-ursus/etc/ursus_errors.xml

2) the chains that the robot have, for example, 

    LEFTARM
    RIGHTARM
    HEAD

3) the motors that make up the chains, for example,

    LEFTARM=leftShoulder1;leftShoulder2;leftShoulder3;leftElbow;leftForeArm;leftWrist1;leftWrist2

4) And, finally, the tip or end effector of the chain, for example, 
    
    LETFTIP=grabPositionHandL.

When you send a target position (with his traslation and rotation), you must indicates the chain that will run the target, and the target traslations and rotations weights. 

This revision of the component includes some new enhancements such as:

1) Executes more than once a target. The inverse kinematic result is not the same if the start point of the effector is the robot's home or a point B near tho the goal point.

2) Executes the traslations without the motors of the wrisht (only for Ursus). This makes possible to move the arm with stiff wrist, and then we can rotate easely the wrist when the end effectos is near the target.


##Configuration parameters
We can connect this component with other components through the port 10220.

Like all the components of Robocomp, inversekinematics needs a configuration file to start. You can see one example in

    etc/configDefinitivo

We can find there the following lines:

    BodyInverseKinematics.Endpoints=tcp -p 10220 								#The port of the component
    CommonBehavior.Endpoints=tcp -p 12207
    JoystickAdapterTopic=tcp -p 12226
    
    JointMotorProxy = jointmotor:tcp -p 20000 									#We need the ursuscommonjoint in
                                                                                #order to move the motors of the robot.
    DifferentialRobotProxy = differentialrobot:tcp  -p 10004 -h localhost		#This is the base of the robot
    OmniRobotProxy = omnirobot:tcp  -p 12238 -h localhost							
    InnerModelManagerProxy = innermodelmanager:tcp  -p 11175 -h localhost		#To manage the InnerModel
    
    InnerModel=/home/robocomp/robocomp/components/robocomp-ursus/etc/ursus.xml	#A model of the robot Ursus

    #Motors of the robot Ursus:
	RIGHTARM=rightShoulder1;rightShoulder2;rightShoulder3;rightElbow;rightForeArm;rightWrist1;rightWrist2
	RIGHTTIP=grabPositionHandR
	LEFTARM=leftShoulder1;leftShoulder2;leftShoulder3;leftElbow;leftForeArm;leftWrist1;leftWrist2
	LETFTIP=grabPositionHandL
	HEAD=head_yaw_joint;head_pitch_joint
	HEADTIP=rgbd_transform
	
	TopicManager.Proxy=IceStorm/TopicManager:default -p 9999
	
	Ice.Warn.Connections=0
	Ice.Trace.Network=0
	Ice.Trace.Protocol=0
	Ice.ACM.Client=10
	Ice.ACM.Server=10

    
##Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd /robocomp/components/robocomp-ursus/components/inversekinematics
    cp etc/config ourConfig
    
After editing the new config file (ourConfig) we can run the component:

    ./bin/inversekinematics ourConfig
    
It's not necessary put --Ice.Config=.