# AGM
echo "AGM (pulsa tecla)"
read a;
echo $a
cd /home/robocomp/AGM
git diff
read a;
git commit -a
git push


# robocomp
echo "robocomp (pulsa tecla)"
read a;
cd /home/robocomp/robocomp/build
git diff
read a;
git commit -a
git push


# robocomp-robolab
echo "robocomp-robolab (pulsa tecla)"
read a;
cd /home/robocomp/robocomp/components/robocomp-robolab
git diff
read a;
git commit -a
git push


# robocomp-ursus
echo "robocomp-ursus (pulsa tecla)"
read a;
cd /home/robocomp/robocomp/components/robocomp-ursus
git diff
read a;
git commit -a
git push


# robocomp-ursus-rockin
echo "robocomp-ursus-rockin (pulsa tecla)"
read a;
cd /home/robocomp/robocomp/components/robocomp-ursus-rockin
git diff
read a;
git commit -a
git push

