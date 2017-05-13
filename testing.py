from matplotlib import pyplot as plt
import random
import math

for k in range(100):
	X = []
	Y = []
	for i in range(10000):
		r, theta = [math.sqrt(random.randint(0,500))*math.sqrt(500), 2*math.pi*random.random()]
		if r > 250:
			r = 2
		x = 500 + r * math.cos(theta) 
		y = 500 + r * math.sin(theta)
		X.append(x)
		Y.append(y)

	plt.plot(X,Y,'r*')
	plt.pause(0.1)
	if i % 3 == 0:
		plt.cla()

