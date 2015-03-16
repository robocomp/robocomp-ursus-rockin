from pylab import *
import os

t = linspace(0,2*pi, num=200)
x = cos(3*t)
y = sin(4*t)
plot(x,y)
plt.show()

file = open("pointsA.txt", "w+")

x = x *2000
y = y *2000
for i in range(len(x)):
	print >> file, x[i], y[i]
	
file.close()