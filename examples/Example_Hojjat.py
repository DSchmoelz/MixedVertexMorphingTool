#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Hojjat - Example 1D Vertex Morphing
#####################################################################

import numpy as np
import sys
import path_setting
sys.path.append(path_setting.path)
from mixedvmtool import *
import matplotlib.pyplot as plt

" Control geometry "
p_j_0 = np.array([0, 1, 2, 3, 4, 5, 4, 7, 8, 5, 2, 3, 4, 3, 2, 1, 0])
# p_j_1 = np.array([0, 2, 4, 8, 4, 0])
# p_j_2 = np.array([0, 4, 8, 4, 0])
# p_j_3 = np.array([0, 8, 0])

x_j_0 = np.linspace(0, 16, len(p_j_0))
# x_j_1 = np.linspace(0, 16, len(p_j_1))
# x_j_2 = np.linspace(0, 16, len(p_j_2))
# x_j_3 = np.linspace(0, 16, len(p_j_3))

control_number_of_nodes = len(p_j_0)
ControlNodeList = []
control_ids = np.arange(control_number_of_nodes)
for i in range(0, control_number_of_nodes):
    ControlNodeList.append(ControlNode(control_ids[i], x_j_0[i], p_j_0[i]))

ControlMesh = Mesh("control")
ControlMesh.AddNodes(ControlNodeList)

# ControlMesh_1 = []
# id = 0
# for i in range(len(p_j_1)):
#     ControlMesh_1.append(Node.Control(id, x_j_1[i], p_j_1[i]))
#     id += 1

# ControlMesh_2 = []
# id = 0
# for i in range(len(p_j_2)):
#     ControlMesh_2.append(Node.Control(id, x_j_2[i], p_j_2[i]))
#     id += 1

# ControlMesh_3 = []
# id = 0
# for i in range(len(p_j_3)):
#     ControlMesh_3.append(Node.Control(id, x_j_3[i], p_j_3[i]))
#     id += 1

r_0 = 1
r_1 = 2
r_2 = 4
r_3 = 8

plt.figure()
plt.plot(x_j_0, p_j_0, '*-', color='lightgrey', label='control geometry')

" Design geometry "
num_design_nodes = (17-1)*1+1
z_i_0 = np.zeros(num_design_nodes)
z_i_1 = np.zeros(num_design_nodes)
z_i_2 = np.zeros(num_design_nodes)
z_i_3 = np.zeros(num_design_nodes)
x_i = np.linspace(0, 16, num_design_nodes)

DesignNodeList = []
design_ids = np.arange(num_design_nodes)
for i in range(0, num_design_nodes):
    DesignNodeList.append(DesignNode(design_ids[i], x_i[i], 0))

DesignMesh = Mesh("design")
DesignMesh.AddNodes(DesignNodeList)

A_0 = ForwardMapping.GaussianIntegration(DesignMesh, ControlMesh, r_0)
A_0.Calculate()
A_1 = ForwardMapping.GaussianIntegration(DesignMesh, ControlMesh, r_1)
A_1.Calculate()
A_2 = ForwardMapping.GaussianIntegration(DesignMesh, ControlMesh, r_2)
A_2.Calculate()
A_3 = ForwardMapping.GaussianIntegration(DesignMesh, ControlMesh, r_3)
A_3.Calculate()
# p_j = np.array([])
# for integration_point in B.IntegrationPoints:
#     p_j = np.append(p_j, integration_point.p)

z_i_0 = A_0.MappingMatrix.dot(p_j_0)
z_i_1 = A_1.MappingMatrix.dot(p_j_0)
z_i_2 = A_2.MappingMatrix.dot(p_j_0)
z_i_3 = A_3.MappingMatrix.dot(p_j_0)



plt.plot(x_i, z_i_0, '-', label='design shape of grid, r = {}'.format(r_0))
plt.plot(x_i, z_i_1, '-', label='design shape of grid, r = {}'.format(r_1))
plt.plot(x_i, z_i_2, '-', label='design shape of grid, r = {}'.format(r_2))
plt.plot(x_i, z_i_3, '-', label='design shape of grid, r = {}'.format(r_3))

plt.legend()
plt.axis()
plt.show()
