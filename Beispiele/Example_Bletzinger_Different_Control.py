#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Bletzinger - Example Filter 1D
#####################################################################

import numpy as np
import sys
import path_setting
sys.path.append(path_setting.path)
from vmtool import *
import matplotlib.pyplot as plt

def control_geometry(x_j):
    if x_j < 0:
        p_j = 3/4 * x_j + 3
    elif x_j > 0:
        p_j = -3/4 * x_j + 3
    else:
        p_j = 3
    return p_j

#control_number_of_nodes = 5
#x_j = np.linspace(-4, 4, control_number_of_nodes)
#p_j = np.zeros(control_number_of_nodes)
x_j = np.array([-4, -2, 0, 0, 0, 2, 4])
control_number_of_nodes = len(x_j)
p_j = np.zeros(len(x_j))

ControlNodeList = []
control_ids = np.arange(control_number_of_nodes)
for i in range(0, control_number_of_nodes):
    p_j[i] = control_geometry(x_j[i])
    ControlNodeList.append(ControlNode(control_ids[i], x_j[i], p_j[i]))

ControlMesh = Mesh("control")
ControlMesh.AddNodes(ControlNodeList)

filter_radius = 4
x_limit = filter_radius + 4
design_number_of_nodes = (x_limit)*1+1
x_i = np.linspace(-x_limit, x_limit, design_number_of_nodes)

DesignNodeList = []
design_ids = np.arange(design_number_of_nodes)
for i in range(0, design_number_of_nodes):
    DesignNodeList.append(DesignNode(design_ids[i], x_i[i], 0))

DesignMesh = Mesh("design")
DesignMesh.AddNodes(DesignNodeList)

plt.figure()
plt.plot(x_j, p_j, '-*', color='lightgrey', label='control polygon')

# Method A with Gaussian Integration
A = ForwardMapping.GaussianIntegration(DesignMesh, ControlMesh, filter_radius)
A.Calculate()
print(A.MappingMatrix)
z_i_A = A.MappingMatrix @ p_j

plt.plot(x_i, z_i_A, '-', label='design shape A, r = {}'.format(filter_radius))

# Method B with Riemann sum
B = ForwardMapping.RiemannSum(DesignMesh, ControlMesh, filter_radius)
B.Calculate()

z_i_B = B.MappingMatrix @ p_j

plt.plot(x_i, z_i_B, '-', label='design shape B, r = {}'.format(filter_radius))

plt.legend()
plt.show()
