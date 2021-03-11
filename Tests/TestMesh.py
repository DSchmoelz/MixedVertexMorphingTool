#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Mesh Test
#####################################################################

import numpy as np
import sys
import path_setting
sys.path.append(path_setting.path)
from vmtool import *
import matplotlib.pyplot as plt

# x_i = np.linspace(-8, 8, 17)
x_i = np.random.rand(10)
np.sort(x_i)
# x_i = np.array([0, 2, 3, 3.5, 4.5, 5, 8, 11, 12, 12.5, 13, 16, 18, 20])
# p_i = np.array([1, 0.7, 2, -1.2, -2.2, -1, 0.4, 1.35, 1.3, 1.25, 1.2, 1.15, 1.1, 1.05, 1, -1, -1])
p_i = np.random.rand(len(x_i))
# p_i = np.array([1, 2, 3, 4, 5, 5, 3, 2, 1, 0, 1, 3, 2, 1])

ids = np.arange(len(x_i))
np.random.shuffle(ids)

NodeList = []

for i in range(0, len(x_i)):
    NodeList.append(ControlNode(ids[i], x_i[i], p_i[i]))

ControlMesh = Mesh("control")
ControlMesh.AddNodes(NodeList)


# look_for_neighbours_of_node_id = 8
# node = ControlMesh.GetNodeWithId(look_for_neighbours_of_node_id)
# print("Look for neighbours of node id {} at x = {}".format(node.id, node.x))
# for neighbour in ControlMesh.GetNodeNeighbours(look_for_neighbours_of_node_id):
#     print("Neighbour id {} at x = {}".format(neighbour.id, neighbour.x))

x_j = np.linspace(0, 1, 1000)
y = np.zeros(1000)

for i in range(len(x_j)):
    y[i] = ControlMesh.GetGeometryAt(x_j[i])

shapefunction = np.zeros([len(x_i), 1000])
for node in ControlMesh.Nodes:
    shape_function_length = ControlMesh.GetNodeShapeFunctionLengths(node.id)
    for i in range(len(x_j)):
        shapefunction[node.id, i] = ShapeFunctions.LinearNodeShapeFunction(x_j[i], node.x, shape_function_length[0], shape_function_length[1])

    plt.plot(x_j, shapefunction[node.id, :], '--', label="shape functions of node id {}".format(node.id))


z_j = np.array(y)
plt.plot(x_j, z_j, '-', label='control geometry with get geometry')
plt.plot(x_i, p_i, '*', label='true control geometry')

nodes_in_filer = ControlMesh.GetNodeInFilterRadius(0.5, 0.2, 0.2)
for node in nodes_in_filer:
    print("Node id {} at {} in filter around x = {} with radius = {}".format(node.id,node.x, 0.5, 0.2))

plt.legend()
plt.show()
