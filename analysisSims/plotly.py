import os
print(os.sys.path)

import numpy as np
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go

import numpy as np
import matplotlib.pyplot as plt
from itertools import repeat  # just for the example

def makesphere(x, y, z, radius, resolution=10):
    """Return the coordinates for plotting a sphere centered at (x,y,z)"""
    u, v = np.mgrid[0:2*np.pi:resolution*2j, 0:np.pi:resolution*1j]
    
    radList = np.random.uniform(3, 5, resolution*2)
    X = radList * np.cos(u)*np.sin(v) + x
    
    print(X)
    Y = radList * np.sin(u)*np.sin(v) + y
    Z = radList * np.cos(v) + z
    return (X, Y, Z)

fig = plt.figure("Spheres")
ax = fig.add_subplot(projection='3d')

X, Y, Z = makesphere(0, 0, 0, 5)
ax.plot_surface(X, Y, Z, color="r")
data = []
data.append(go.Surface(x=X, y=Y, z=Z, opacity=0.5))
fig = go.Figure(data=data)
fig.show()

# for x, y, z, radius, in zip(*repeat(np.arange(1,4),3), [.1, .5, .75]):
    