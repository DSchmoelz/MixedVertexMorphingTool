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
from vmtool import *
import matplotlib.pyplot as plt



control_number_of_nodes = 17
p_j = np.array([0, 1, 2, 3, 4, 5, 4, 7, 8, 5, 2, 3, 4, 3, 2, 1, 0])
x_j = np.linspace(0, 16, control_number_of_nodes)

ControlNodeList = []
control_ids = np.arange(control_number_of_nodes)
np.random.shuffle(control_ids)
for i in range(0, control_number_of_nodes):
    ControlNodeList.append(ControlNode(control_ids[i], x_j[i], p_j[i]))

ControlMesh = Mesh("control")
ControlMesh.AddNodes(ControlNodeList)


design_number_of_nodes = (control_number_of_nodes-1)*4 + 1
x_i = np.linspace(0, 16, design_number_of_nodes)

DesignNodeList = []
design_ids = np.arange(design_number_of_nodes)
np.random.shuffle(design_ids)
for i in range(0, design_number_of_nodes):
    DesignNodeList.append(DesignNode(design_ids[i], x_i[i], 0))

DesignMesh = Mesh("design")
DesignMesh.AddNodes(DesignNodeList)


plt.figure()

plt.plot(x_j, p_j, '-', color='lightgrey', label='control polygon')

r = np.array([0.1 ,1, 2, 4, 8, 16])
#r = np.array([1])
n = np.array([1, 2, 4, 8])
shape_function_span = 1
# filter_radius = 1

for filter_radius in r:
    # filter_radius = 4
    # A = ForwardMapping.Summation(DesignMesh, ControlMesh, filter_radius, shape_function_span)
    B = ForwardMapping.GaussianIntegration(DesignMesh, ControlMesh, filter_radius)

    # A.Calculate()
    B.Calculate()

    # z_i_A = A.MappingMatrix.dot(p_j)
    z_i_B = B.MappingMatrix.dot(p_j)

    # for x in range(0, number_of_nodes):
    #     Nodes[x].p = z_i[x]

    # p_i = np.linalg.inv(A.MappingMatrix).dot(z_i)

    # plt.plot(x_i, z_i_A, '-', label='design shape A, r = {}'.format(filter_radius))
    plt.plot(x_i, z_i_B, '-', label='design shape B, r = {}'.format(filter_radius))


# plt.plot(x, p_i, '-', label='control polygon backward computed')
plt.legend()
plt.show()
