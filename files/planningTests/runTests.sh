planningTest="navigation/hallToPatio"
echo "Next: $planningTest"
echo "Press enter to run test"
read c
cd $planningTest
agglplan ../../../../etc/domain.aggl initialModel.xml target.xml
cd -

planningTest="perception/findGranny"
echo "Next: $planningTest"
echo "Press enter to run test"
read c
cd $planningTest
agglplan ../../../../etc/domain.aggl initialModel.xml target.xml
cd -

planningTest="grasp"
echo "Next: $planningTest 1"
echo "Press enter to run test"
read c
cd $planningTest
agglplan ../../../etc/domain.aggl initialModel1.xml target1.xml
cd -

planningTest="grasp"
echo "Next: $planningTest 2"
echo "Press enter to run test"
read c
cd $planningTest
agglplan ../../../etc/domain.aggl initialModel2.xml target2.xml
cd -




