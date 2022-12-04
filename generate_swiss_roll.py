import matplotlib.pyplot as plt
from sklearn import manifold, datasets
import numpy as np

sr_points, sr_color = datasets.make_swiss_roll(n_samples=500, noise=0.0, random_state=0)
"""
L=[]
C=[]
for i in range(len(sr_points)):
    if sr_points[i][0]>-3 or sr_points[i][1]<7 or sr_points[i][1]>13:
        L.append(sr_points[i])
        C.append(sr_color[i])
sr_points = np.array(L)
sr_color = np.array(C)
"""
##
from mpl_toolkits.mplot3d import Axes3D
##

fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")
fig.add_axes(ax)
ax.scatter(
    sr_points[:, 0], sr_points[:, 1], sr_points[:, 2], c=sr_color, s=50, alpha=0.8
)
ax.set_title("Swiss Roll in Ambient Space")
ax.view_init(azim=-66, elev=12)
_ = ax.text2D(0.8, 0.05, s="n_samples=1500", transform=ax.transAxes)
plt.show()
## writ off file

PATH = "C:/Users/hugob/Desktop/swiss_roll.txt"

f = open(PATH,'w')
f.write('OFF\n')
f.write(f'{len(sr_points)} 0 0\n')
for x in sr_points:
    x = [str(round(y,5)) for y in x]
    f.write(" ".join(x) + "\n")

f.close()

##generate petal
import numpy as np
n=1000
L=[]
C=[]
while len(L) < n:
    alpha = np.random.random() * 2 * np.pi
    beta = np.random.random() * 2 * np.pi
    pt = [np.cos(alpha)*np.cos(beta), np.sin(alpha)*np.cos(beta), np.sin(beta)]
    indicator = 2*abs(alpha%(np.pi/2)-np.pi/4) + 5*abs(0.5*np.sin(beta))**4
    if pt[2] < 0.65 and indicator < 0.9:
        r = 1 + 0.2*(0.5-np.random.random())
        pt[0] *= r
        pt[1] *= r
        pt[2] *= r
        L.append(pt)
        C.append(pt[2] * 10 + 4)

sr_points = np.array(L)
sr_color = np.array(C)



##decode

PATH2 = "C:/Users/hugob/Desktop/res.txt"
f = open(PATH2,'r')
L = []
for x in f.readlines():
    l = [float(k) for k in x.split(" ") if k!=""]
    L.append(l)
print(len(L))
L = np.array(L)
X = L[:,0]
Y = L[:,1]
plt.plot(X,L,'bo')
plt.show()





























