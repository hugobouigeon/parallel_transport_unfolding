import matplotlib.pyplot as plt
import numpy as np

PATH = "./src/data/results.txt"
f = open(PATH,'r')
L = []
for x in f.readlines():
    l = [float(k) for k in x.split(" ") if k!=""]
    L.append(l)
print(len(L))
L = np.array(L)
X = L[:,0]
Y = L[:,1]
plt.plot(X,Y,'o')
plt.axis('equal')
plt.show()
f.close()