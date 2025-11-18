#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# RunVertexMorphing
#####################################################################

import numpy as np
import sys
import path_setting
sys.path.append(path_setting.path)
from mixedvmtool import *
import matplotlib.pyplot as plt


# Control Field
control_number_of_nodes = 101
p_j = np.zeros(int(control_number_of_nodes))
x_j = np.linspace(0, 100, control_number_of_nodes)
for i in range(control_number_of_nodes):
    p_j[i] = x_j[i] / x_j[-1]


ControlNodeList = []
control_ids = np.arange(control_number_of_nodes)
for i in range(0, control_number_of_nodes):
    ControlNodeList.append(ControlNode(control_ids[i], x_j[i], p_j[i]))
ControlMesh = Mesh("control")
ControlMesh.AddNodes(ControlNodeList)

# Design Nodes
design_number_of_nodes = 101
x_i = np.linspace(0, 100, design_number_of_nodes)
DesignNodeList = []
design_ids = np.arange(design_number_of_nodes)
for i in range(0, design_number_of_nodes):
    DesignNodeList.append(DesignNode(design_ids[i], x_i[i], 0))
DesignMesh = Mesh("design")
DesignMesh.AddNodes(DesignNodeList)

figure, axis = plt.subplots(1, figsize=[16.0,9.0])

# axis.plot(x_j, p_j, '-', color='lightgrey', label='control polygon')

r = 30
# Method A with Riemann sum
settings_A = {
    "filter_radius": r,
    "integration": "RiemannSum",
    "scaling": "none"
}
A = VertexMorphingParameterization.VertexMorphing(DesignMesh, ControlMesh, settings_A)
A.Calculate()

settings_B = {
    "filter_radius": r,
    "integration": "RiemannSum",
    "scaling": "none"
}
B = VertexMorphingParameterization.VertexMorphing(DesignMesh, ControlMesh, settings_A)
B.Calculate()

print(f"mapping matrix:\n{A.MappingMatrix}")

# Physical Bounds
physical_bound = np.zeros(int(control_number_of_nodes))
for node in ControlMesh.Nodes:
    index = ControlMesh.GetNodeIndex(node.id)
    # physical_bound[index] = node.x / x_j[-1]
    # physical_bound[index] = -(node.x-5)**2 / 25 + 1
    physical_bound[index] = -(node.x-50)**2 / (50*50) + 1

# Control Bounds - Determine with min bound in radius
bound = np.full(int(control_number_of_nodes), 1e8)
# bound = physical_bound.copy()
for node in ControlMesh.Nodes:
    index = ControlMesh.GetNodeIndex(node.id)
    neighbour_nodes = ControlMesh.GetNodeInFilterRadius(node.x, r, r)
    for neigbour_node in neighbour_nodes:
        neighbour_index = ControlMesh.GetNodeIndex(neigbour_node.id)
        physical_bound_i = physical_bound[neighbour_index]
        if physical_bound_i < bound[index]:
            bound[index] = physical_bound_i
        # if physical_bound_i == 0:
        #     bound[index] = physical_bound_i

# for node in ControlMesh.Nodes:
#     if node.x >= r:
#         index = ControlMesh.GetNodeIndex(node.id)
#         bound[index] += 0.1

# bound = physical_bound
# bound[:] += 0.05
    # sum_of_weights = 0
    # for neigbour_node in neighbour_nodes:
    #     neighbour_index = ControlMesh.GetNodeIndex(neigbour_node.id)
    #     weight = LinearFilter(node.x, neigbour_node.x, r)
    #     print(f"weight: {weight}")
    #     sum_of_weights += weight
    #     print(f"sum_of_weights: {weight}")

    # for neigbour_node in neighbour_nodes:
    #     neighbour_index = ControlMesh.GetNodeIndex(neigbour_node.id)
    #     weight = LinearFilter(node.x, neigbour_node.x, r)
    #     physical_bound_i = physical_bound[neighbour_index]
    #     if weight > 0:
    #         physical_bound_i *= weight / sum_of_weights
    #         print(f"physical_bound_i: {physical_bound_i}")
    #         if physical_bound_i < bound[index]:
    #             bound[index] = physical_bound_i

axis.plot(x_j, physical_bound, '-', color='grey', label='control variable')
# axis.plot(x_j, bound, '-', color='red', label='control bound')


physical_variable = A.MapUpdate(physical_bound)

axis.plot(x_i, physical_variable, '-', label='physical variable, r = {}'.format(r))
axis.set_ylim([0,1.2])
plt.legend()
plt.show()
