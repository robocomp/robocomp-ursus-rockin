DOMAINNAME=domain_basic.aggl

echo
echo "######################################################################"
planningTest="answerDoor/unknown"
echo "Next: $planningTest"
cd $planningTest
agglplan ../../../../etc/$DOMAINNAME initialModel.xml targetModel.xml
echo "######################################################################"
echo
cd -
sleep 1

