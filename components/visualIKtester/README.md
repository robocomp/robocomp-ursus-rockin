visualIKtester
===============================
##What is Visual Inverse Kinematic tester (visualIKtester)?
In the [`visualik` component](https://github.com/robocomp/robocomp-ursus/tree/master/components/visualik/README.md) we explained the problems of the inverse kinematics of a robot and the solution provided through the visual feedback with the `visualik` component.

In order to prove the implementation of this solution, we have implemented this component that tests the `visualik` behavior in various ways:

1. Mainly this component sends `visualik` one POSE6D target with a pose and a certain rotation (if you press the "rotation" option) through the first tab of the interface.
2. With the second tab you can send an ALIGNAXIS or an ADVACEAXIS target but this option is not fully developed.
3. You can send 10 random POSE6D targets to `visualik`. This option is not collected at the interface, if not inside the code.

![Alt text](https://github.com/robocomp/robocomp-ursus-rockin/blob/master/components/visualIKtester/etc/1tester.png)

##Configuration parameters
This component can interactuate with the `inversekinematics` component and with the `visualik` component, only changing the port of one of the settings. So in the `visualIKtester` configuration whe must put the next data:
1. We need the controller of the joints, in order to move the kinematic chain when we recive the resonse of the `inversekinematics` compoenent
2. We need to connect the tester with the `inversekinematics`component or with the `visualik`compoenent.

So we need a config like this:

    # Proxies for required interfaces
    JointMotorProxy = jointmotor:tcp -h localhost -p 20000
    # INVERSE KINEMATICS COMPONENT:
    #InverseKinematicsProxy = inversekinematics:tcp -h localhost -p 10220
    # VISUAL INVERSE KINEMATICS COMPONENT:
    InverseKinematicsProxy = inversekinematics:tcp -h localhost -p 10240
    
    # This property is used by the clients to connect to IceStorm.
    TopicManager.Proxy=IceStorm/TopicManager:default -p 9999
    Ice.Warn.Connections=0
    Ice.Trace.Network=0
    Ice.Trace.Protocol=0
    Ice.ACM.Client=10
    Ice.ACM.Server=10
    
##Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd /robocomp/components/robocomp-ursus-rockin/components/visualIKtester
    cp etc/config ourConfig
    
After editing the new config file (ourConfig) we can run the component:

    python src/mueveBrazo.py etc/ourConfig
    
It's not necessary put the command `--Ice.Config=`.
