#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Test Blending Functions
#####################################################################

import numpy as np
import sys
import path_setting
sys.path.append(path_setting.path)
from vmtool import *
import matplotlib.pyplot as plt

## Design Geometry
filter_radius = 4
x_limit = filter_radius + 4

number_of_nodes = 2*(x_limit)+1
x_i = np.linspace(-x_limit, x_limit, number_of_nodes)
z_i = 2
node_list = []
design_ids = np.arange(number_of_nodes)

for i in range(0, number_of_nodes):
    node_list.append(DesignNode(design_ids[i], x_i[i], z_i))

mesh = Mesh("design")
mesh.AddNodes(node_list)

## Compute Blending
blending_node_ids = []
for node in mesh.Nodes:
    if node.x >= -2 and node.x <= 2:
        blending_node_ids.append(node)

blending_function = mesh.ComputeBlendingFunction(blending_node_ids, 2)

# Plot
figure, axis = plt.subplots(figsize=[8.0,6.0])
# axis[0].plot(x_j, p_j, '-*', color='lightgrey', label='target shape')

shape = mesh.GetShapeZ()
blending_shape = blending_function * shape

axis.plot(mesh.GetNodeCoordinatesX(), shape, '-', label='shape')
axis.plot(mesh.GetNodeCoordinatesX(), blending_function, '-', label='blending function')
axis.plot(mesh.GetNodeCoordinatesX(), blending_shape, '-', label='blending shape')
axis.axis('equal')
axis.legend()

figure.show()
plt.show()