import sys, copy
import numpy

def generateErrorsXML(input_file, output_file, t_err, r_err, f_err):
	fields = dict()
	fields["tx"] = stdDevT
	fields["ty"] = stdDevT
	fields["tz"] = stdDevT
	fields["rx"] = stdDevR
	fields["ry"] = stdDevR
	fields["rz"] = stdDevR
	fields["focal"] = stdDevF
	print fields

	for line in entrada.readlines():
		lineOut = copy.deepcopy(line)
		if "@error".lower() in line.lower():
			if "<transform" in line:
				lineOut = cambiaError(line, fields )
			elif "<translation" in line:
				lineOut = cambiaError(line, fields )
			elif "<rotation" in line:
				lineOut = cambiaError(line, fields )
			elif "<rgbd" in line:
				lineOut = cambiaError(line, fields )
			else:
				print 'whaaaat', line
		salida.write(lineOut)
	entrada.close()
	salida.close()


def cambiaError(line, fields):
	ret = ''
	s = line.split('"')
	first = ''
	for i in xrange(len(s)):
		if i%2 == 1:
			done = False
			for field in fields:
				if s[i-1].endswith(field+'='):
					if len(s)>i and "@error".lower() in s[i].lower():
						XXX = s[i]
						base = s[i].lower().split("@error")[0]
						if base == '':
							base = 0
						else:
							base = float(base)
						fl = s[i-1][:-1].strip()
						#print 'field', fl
						ret += first + str(base+numpy.random.normal(0., fields[fl], 1)[0])
						first = '"'
						done = True
						break
			if not done:
				ret += first + s[i]
				first = '"'
		else:
			ret += first + s[i]
			first = '"'
	return ret




if __name__ == '__main__':
	if len(sys.argv) < 6:
		print 'python', sys.argv[0], 'input.xml output.xml err_translation err_rotation err_focal'
		sys.exit(-1)
	else:
		entrada = open(sys.argv[1], "r")
		salida  = open(sys.argv[2], "w")
		stdDevT = float(sys.argv[3])
		stdDevR = float(sys.argv[4])
		stdDevF = float(sys.argv[5])
		generateErrorsXML(entrada, salida, stdDevT, stdDevR, stdDevF)


