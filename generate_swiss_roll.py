import matplotlib.pyplot as plt
from sklearn import manifold, datasets


sr_points, sr_color = datasets.make_swiss_roll(n_samples=1500, noise=0.2, random_state=0)


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

## writ off file

PATH = "C:/Users/hugob/Desktop/swiss_roll.txt"

f = open(PATH,'w')
f.write('OFF\n')
f.write(f'{len(sr_points)} 0 0\n')
for x in sr_points:
    x = [str(round(y,5)) for y in x]
    f.write(" ".join(x) + "\n")

f.close()