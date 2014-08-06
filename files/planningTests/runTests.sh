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




