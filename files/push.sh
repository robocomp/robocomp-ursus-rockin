#!/usr/bin/env bash

# AGM
echo "AGM (pulsa tecla)"
read a;
echo $a
cd /home/robocomp/AGM
git diff
echo "(eso era el diff, pulsa una tecla)"
read a;
git commit -a
if [ $? -ne 1 ]; then
	git push
else
	echo "No hay nada que subir"
fi


# robocomp
echo "robocomp (pulsa tecla)"
read a;
cd /home/robocomp/robocomp
git diff
echo "(eso era el diff, pulsa una tecla)"
read a;
git commit -a
if [ $? -ne 1 ]; then
	git push
else
	echo "No hay nada que subir"
fi


# robocomp-robolab
echo "robocomp-robolab (pulsa tecla)"
read a;
cd /home/robocomp/robocomp/components/robocomp-robolab
git diff
echo "(eso era el diff, pulsa una tecla)"
read a;
git commit -a
if [ $? -ne 1 ]; then
	git push
else
	echo "No hay nada que subir"
fi


# robocomp-ursus
echo "robocomp-ursus (pulsa tecla)"
read a;
cd /home/robocomp/robocomp/components/robocomp-ursus
git diff
echo "(eso era el diff, pulsa una tecla)"
read a;
git commit -a
if [ $? -ne 1 ]; then
	git push
else
	echo "No hay nada que subir"
fi


# robocomp-ursus-rockin
echo "robocomp-ursus-rockin (pulsa tecla)"
read a;
cd /home/robocomp/robocomp/components/robocomp-ursus-rockin
git diff
echo "(eso era el diff, pulsa una tecla)"
read a;
git commit -a
if [ $? -ne 1 ]; then
	git push
else
	echo "No hay nada que subir"
fi

