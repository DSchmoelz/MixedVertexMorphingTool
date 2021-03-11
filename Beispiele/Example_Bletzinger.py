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

# control_number_of_nodes = 9
# p_j = np.array([0, 0.75, 1.5, 2.25, 3, 2.25, 1.5, 0.75, 0])
# x_j = np.linspace(-4, 4, control_number_of_nodes)
control_number_of_nodes = 3
p_j = np.array([0, 3, 0])
x_j = np.linspace(-4, 4, control_number_of_nodes)

ControlNodeList = []
control_ids = np.arange(control_number_of_nodes)
for i in range(0, control_number_of_nodes):
    ControlNodeList.append(ControlNode(control_ids[i], x_j[i], p_j[i]))

ControlMesh = Mesh("control")
ControlMesh.AddNodes(ControlNodeList)

filter_radius = 4
x_limit = filter_radius + 4
design_number_of_nodes = (x_limit)*4+1
x_i = np.linspace(-x_limit, x_limit, design_number_of_nodes)

DesignNodeList = []
design_ids = np.arange(design_number_of_nodes)
for i in range(0, design_number_of_nodes):
    DesignNodeList.append(DesignNode(design_ids[i], x_i[i], 0))

DesignMesh = Mesh("design")
DesignMesh.AddNodes(DesignNodeList)

plt.figure()
plt.plot(x_j, p_j, '-*', color='lightgrey', label='control polygon')


A = ForwardMapping.GaussianIntegration(DesignMesh, ControlMesh, filter_radius)
A.Calculate()

z_i_A = A.MappingMatrix @ p_j

plt.plot(x_i, z_i_A, '-', label='design shape A, r = {}'.format(filter_radius))

plt.legend()
plt.show()
