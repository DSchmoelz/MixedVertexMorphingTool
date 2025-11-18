#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Example - Bletzinger Varying filters (Fig. 14)
#####################################################################

import numpy as np
import sys
import path_setting
sys.path.append(path_setting.path)
from mixedvmtool import *
import matplotlib.pyplot as plt

p_j = np.array([1, 0.95, 0.9, 0.85, 0.8, 0.75, 0.7, 2, -1.2, -2.2, -1, 0.4, 1.35, 1.3, 1.25, 1.2, 1.15, 1.1, 1.05, 1, -1, -1, -1, -1])
control_number_of_nodes = len(p_j)
x_j = np.linspace(0, control_number_of_nodes-1, control_number_of_nodes)

ControlNodeList = []
control_ids = np.arange(control_number_of_nodes)
for i in range(0, control_number_of_nodes):
    ControlNodeList.append(ControlNode(control_ids[i], x_j[i], p_j[i]))

ControlMesh = Mesh("control")
ControlMesh.AddNodes(ControlNodeList)


design_number_of_nodes = (control_number_of_nodes-1)*1 + 1
x_i = np.linspace(0, control_number_of_nodes-1, design_number_of_nodes)

DesignNodeList = []
design_ids = np.arange(design_number_of_nodes)
for i in range(0, design_number_of_nodes):
    DesignNodeList.append(DesignNode(design_ids[i], x_i[i], 0))

DesignMesh = Mesh("design")
DesignMesh.AddNodes(DesignNodeList)

plt.figure()
plt.plot(x_j, p_j, '-*', color='lightgrey', label='control polygon')

filter_radius = 11

# Method A with Riemann sum
settings_A = {
    "filter_radius": filter_radius,
    "integration": "RiemannSum",
    "scaling": "none"
}
A = VertexMorphingParameterization.VertexMorphing(DesignMesh, ControlMesh, settings_A)
A.Calculate()
z_i_A = A.MapUpdate(p_j)
print(A.MappingMatrix)
plt.plot(x_i, z_i_A, '-', label='design shape A, r = {}'.format(filter_radius))

plt.legend()
plt.axis()
plt.show()
