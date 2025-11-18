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


factor = 1e4
control_number_of_nodes = int(10 * factor + 1)
p_j = np.zeros(int(control_number_of_nodes))
p_j[int(5*factor)] = factor
x_j = np.linspace(0, 10, control_number_of_nodes)

ControlNodeList = []
control_ids = np.arange(control_number_of_nodes)
np.random.shuffle(control_ids)
for i in range(0, control_number_of_nodes):
    ControlNodeList.append(ControlNode(control_ids[i], x_j[i], p_j[i]))

ControlMesh = Mesh("control")
ControlMesh.AddNodes(ControlNodeList)


design_number_of_nodes = 11
x_i = np.linspace(0, 10, design_number_of_nodes)

DesignNodeList = []
design_ids = np.arange(design_number_of_nodes)
np.random.shuffle(design_ids)
for i in range(0, design_number_of_nodes):
    DesignNodeList.append(DesignNode(design_ids[i], x_i[i], 0))

DesignMesh = Mesh("design")
DesignMesh.AddNodes(DesignNodeList)


figure, axis = plt.subplots(1, figsize=[8.0,6.0])
axis.set_ylim([0,1])
control_update = np.zeros(int(control_number_of_nodes))
control_update[int(5*factor)] = factor
axis.plot(x_j, control_update, '-', color='lightgrey', label='control polygon')

# r = np.array([0.1 ,1, 2, 4, 8, 16])
r = np.array([4])
n = np.array([1, 2, 4, 8])
shape_function_span = 1
# filter_radius = 1

for filter_radius in r:
    # Method A with Riemann sum
    settings_A = {
        "filter_radius": filter_radius,
        "integration": "RiemannSum",
        "scaling": "none"
    }
    # filter_radius = 4
    A = VertexMorphingParameterization.VertexMorphing(DesignMesh, ControlMesh, settings_A)
    # B = ForwardMapping.GaussianIntegration(DesignMesh, ControlMesh, filter_radius)

    A.Calculate()
    # B.Calculate()

    design_update = A.MapUpdate(control_update)
    z_i_A = design_update
    # z_i_A = A.MapUpdate.dot(p_j)
    # z_i_B = B.MappingMatrix.dot(p_j)

    # for x in range(0, number_of_nodes):
    #     Nodes[x].p = z_i[x]

    # p_i = np.linalg.inv(A.MappingMatrix).dot(z_i)


    axis.plot(x_i, z_i_A, '-', label='design shape A, r = {}'.format(filter_radius))

    # plt.plot(x_i, z_i_B, '-', label='design shape B, r = {}'.format(filter_radius))


# plt.plot(x, p_i, '-', label='control polygon backward computed')
plt.legend()
plt.show()
