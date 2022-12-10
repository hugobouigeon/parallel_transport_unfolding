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
    if pt[2] < 0.65 and indicator < 0.9 or pt[2] < 0.-0.95:
        r = 1 + 0.0*(0.5-np.random.random())
        pt[0] *= r
        pt[1] *= r
        pt[2] *= r
        L.append(pt)
        C.append(pt[2] * 10 + 4)

sr_points = np.array(L)
sr_color = np.array(C)

## plane
n=1000
L=[]
C=[]

for i in range(30):
    for j in range (30):
        L.append([i*0.1,j*0.1,(i*0.1)**2 - (j*0.1)**2 ])
        C.append(i / 3 + 4)
sr_points = np.array(L)
sr_color = np.array(C)


##decode

PATH2 = "C:/Users/hugob/Desktop/Projet INF574/parallel_transport_unfolding/src/data/results.txt"
f = open(PATH2,'r')
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

## goundtruth flower

PATH3 = "C:/Users/hugob/Desktop/flower.txt"
f = open(PATH3,'r')
L = []
for x in f.readlines():
    l = [float(k) for k in x.split(" ") if k!=""]
    L.append(l)
print(len(L))
n = Len(L)
L = np.array(L)
X = L[:,0]
Y = L[:,1]
Z = L[:,2]
X1 = [X[i]*np.arcsin(Z[i]-0.5) for i in range(n)]
Y1 = [Y[i]*np.arcsin(Z[i]-0.5) for i in range(n)]
plt.plot(X1,Y1,'o')
plt.axis('equal')
plt.show()
f.close()
## generate world map cloud

import cv2
image = cv2.imread("C:/Users/hugob/Desktop/nicemap.png")
image = image[:,:]
width = len(image[0])
height = len(image)
stride = 8
#print(len([x for x in image[100] if (x[0]>200 and x[1] > 200 and x[2]>200)]))
#plt.matshow(image[:,:,2])
#plt.show()

L=[]
C=[]
for w in range(height//stride):
    for h in range(width//stride-1):
        x = image[w*stride][h*stride]
        y = image[w*stride+1][h*stride+10]
        if ((x[2]>220 or y[2]>220)): #and (h*stride<700 or (w*stride < 250 or w*stride > 300 ))):# or h == 30 or (w == 25 and h>30)):
            alpha = h / width * stride * 2 * np.pi
            beta = np.pi/2 - w / height * stride * np.pi
            pt = [np.cos(alpha)*np.cos(beta), np.sin(alpha)*np.cos(beta), np.sin(beta)]
            #pt = [w,h,0]
            r = 0.03*(0.5-np.random.random())
            pt[0] += r
            pt[1] += r
            pt[2] += r
            L.append(pt)
            C.append(pt[2] * 10 + 4)

sr_points = np.array(L)
sr_color = np.array(C)


fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")
fig.add_axes(ax)
ax.scatter(
    sr_points[:, 0], sr_points[:, 1], sr_points[:, 2], c=sr_color, s=50, alpha=0.8
)
ax.set_title("the earth")
ax.view_init(azim=-66, elev=12)
_ = ax.text2D(0.8, 0.05, s=f"n_samples={len(L)}", transform=ax.transAxes)
plt.show()













