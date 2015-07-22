import sys, os

#P: (266 824 372 0 0 3.1416) ErrorVisual_T:11.8139 ErrorVisual_R:0.00256502 ErrorDirecto_T:0.000183496 ErrorDirecto_R: 2.98181e-08 END: 8 WHY?: LOW_ERROR
if __name__ == '__main__': 
	if len(sys.argv)<2:
		print 'Usage\n\t', sys.argv[0], ' datos.xml'
		sys.exit(-1)
		
	f = open(sys.argv[1])
	
	linea = f.readline()
	error_vt = 0.0
	error_vr = 0.0
	error_dt = 0.0
	error_dr = 0.0
	
	i=0
	while linea !="":
		step1 = linea.split(":");
		step2_1 = step1[2].split(" ")
		step2_2 = step1[3].split(" ")
		step3_1 = step1[4].split(" ")
		step3_2 = step1[5].split(" ")
#		print step3_1, step3_2

		error_vt = error_vt + float(step2_1[0])
		error_vr = error_vr + float(step2_2[0])
		
		error_dt = error_dt + float(step3_1[0])
		error_dr = error_dr + float(step3_2[1])
		linea = f.readline()
		i=i+1
		
	mediavT = error_vt/i
	mediavR = error_vr/i
	mediadT = error_dt/i
	mediadR = error_dr/i

	print "Iteraciones: ",i, "\n    Media visualT: ",mediavT," frente a ",mediadT
	print "   Media visualR: ", mediavR, " frente a ", mediadR
	
	f.close()
