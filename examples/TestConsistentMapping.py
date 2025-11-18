#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Test Consistent Mapping
# Constant variation in control field results in contant shape update
#####################################################################

import numpy as np
import sys
import path_setting
sys.path.append(path_setting.path)
from mixedvmtool import *
import matplotlib.pyplot as plt

## Control = Design
filter_radius = 4
x_limit = filter_radius + 4
number_of_nodes = 2*(x_limit)+1
x_i = np.linspace(-x_limit, x_limit, number_of_nodes)

## Control Geometry
p_j = 0
ControlNodeList = []
control_ids = np.arange(number_of_nodes)
for i in range(0, number_of_nodes):
    ControlNodeList.append(ControlNode(control_ids[i], x_i[i], p_j))

ControlMesh = Mesh("control")
ControlMesh.AddNodes(ControlNodeList)

## Design Geometry
z_i = 0
node_list = []
design_ids = np.arange(number_of_nodes)

for i in range(0, number_of_nodes):
    node_list.append(DesignNode(design_ids[i], x_i[i], z_i))

DesignMesh = Mesh("design")
DesignMesh.AddNodes(node_list)

vm_settings = {
    "filter_radius": filter_radius,
    "integration": "RiemannSum",
    "scaling": "none"
}
VM_Mapper = VertexMorphingParameterization.VertexMorphing(DesignMesh, ControlMesh, vm_settings)
VM_Mapper.Calculate()

control_update = np.ones(len(ControlMesh.Nodes))
design_update =VM_Mapper.MapUpdate(control_update)

final_shape = DesignMesh.GetShapeZ() + design_update

# Plot
figure, axis = plt.subplots(figsize=[8.0,6.0])

axis.plot(DesignMesh.GetNodeCoordinatesX(), DesignMesh.GetShapeZ(), '-', label='inital shape')
axis.plot(DesignMesh.GetNodeCoordinatesX(), final_shape, '-', label='final shape')
axis.plot(DesignMesh.GetNodeCoordinatesX(), control_update, linestyle=":", marker='o', label='control update')
axis.axis('equal')
axis.legend()

figure.show()
plt.show()