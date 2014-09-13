echo
echo "######################################################################"
planningTest="navigation/hallToPatio"
echo "Next: $planningTest"
echo "Press enter to run test"
sleep 2
cd $planningTest
agglplan ../../../../etc/domain.aggl initialModel.xml target.xml
cd -


echo
echo "######################################################################"
planningTest="perception/findGranny"
echo "Next: $planningTest"
echo "Press enter to run test"
sleep 2
cd $planningTest
agglplan ../../../../etc/domain.aggl initialModel.xml target.xml
cd -


echo
echo "######################################################################"
planningTest="grasp"
echo "Next: $planningTest 1"
echo "Press enter to run test"
sleep 2
cd $planningTest
agglplan ../../../etc/domain.aggl initialModel1.xml target1.xml
cd -


echo
echo "######################################################################"
planningTest="grasp"
echo "Next: $planningTest 2"
echo "Press enter to run test"
sleep 2
cd $planningTest
agglplan ../../../etc/domain.aggl initialModel2.xml target2.xml
cd -


echo
echo "######################################################################"
planningTest="deliver/deliverKnown"
echo "Next: $planningTest 0"
echo "Press enter to run test"
sleep 2
cd $planningTest
agglplan ../../../../etc/domain.aggl initialModel.xml target0.xml
cd -


echo
echo "######################################################################"
planningTest="deliver/deliverKnown"
echo "Next: $planningTest 1"
echo "Press enter to run test"
sleep 2
cd $planningTest
agglplan ../../../../etc/domain.aggl initialModel.xml target1.xml
cd -


echo
echo "######################################################################"
planningTest="deliver/deliverKnown"
echo "Next: $planningTest 2"
echo "Press enter to run test"
sleep 2
cd $planningTest
agglplan ../../../../etc/domain.aggl initialModel.xml target2.xml
cd -


echo
echo "######################################################################"
planningTest="hri/coffee"
echo "Next: $planningTest"
echo "Press enter to run test"
sleep 2
cd $planningTest
agglplan ../../../../etc/domain.aggl initialModel.xml target0.xml
cd -

echo
echo "######################################################################"
planningTest="hri/coffee"
echo "Next: $planningTest"
echo "Press enter to run test"
sleep 2
cd $planningTest
agglplan ../../../../etc/domain.aggl initialModel.xml target1.xml
cd -

echo
echo "######################################################################"
planningTest="hri/coffee"
echo "Next: $planningTest"
echo "Press enter to run test"
sleep 2
cd $planningTest
agglplan ../../../../etc/domain.aggl initialModel.xml target2.xml
cd -

echo
echo "######################################################################"
planningTest="hri/coffee"
echo "Next: $planningTest"
echo "Press enter to run test"
sleep 2
cd $planningTest
agglplan ../../../../etc/domain.aggl initialModel.xml target3.xml
cd -

echo
echo "######################################################################"
planningTest="hri/coffee"
echo "Next: $planningTest"
echo "Press enter to run test"
sleep 2
cd $planningTest
agglplan ../../../../etc/domain.aggl initialModel.xml target4.xml
cd -



