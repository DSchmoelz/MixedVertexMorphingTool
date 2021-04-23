#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# TestTargetResponse
#####################################################################

import numpy as np
import sys
import path_setting
sys.path.append(path_setting.path)
from vmtool import *
import matplotlib.pyplot as plt

def target_geometry(x_j):
    if x_j < 0:
        p_j = 3/4 * x_j + 3
    elif x_j > 0:
        p_j = -3/4 * x_j + 3
    else:
        p_j = 3
    return p_j

target_number_of_nodes = 3
x_j = np.linspace(-4, 4, target_number_of_nodes)
p_j = np.zeros(target_number_of_nodes)

TargetNodeList = []
target_ids = np.arange(target_number_of_nodes)
for i in range(0, target_number_of_nodes):
    p_j[i] = target_geometry(x_j[i])
    TargetNodeList.append(ControlNode(target_ids[i], x_j[i], p_j[i]))

TargetMesh = Mesh("target")
TargetMesh.AddNodes(TargetNodeList)
print(TargetMesh.GetGeometryAt(-4))
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
plt.plot(x_j, p_j, '-*', color='lightgrey', label='target polygon')

Response = TargetGeometryResponse(DesignMesh, TargetMesh)

Response.Calculate()

print("response value: {}".format(Response.Value))
print("response gradients: {}".format(Response.Gradients))
print(DesignMesh.GetNodeCoordinatesX())

A = ForwardMapping.GaussianIntegration(DesignMesh, DesignMesh, filter_radius)
A.Calculate()

B = ForwardMapping.RiemannSum(DesignMesh, DesignMesh, filter_radius)
B.Calculate()

mapped_gradients_a = A.MappingMatrix.transpose() @ Response.Gradients
mapped_gradients_b = B.MappingMatrix.transpose() @ Response.Gradients

print("mapped gradients Gaussian: {}".format(mapped_gradients_a))
print("mapped gradients RiemannSum: {}".format(mapped_gradients_b))

step_size = 0.5
delta_p_a = - step_size * mapped_gradients_a
delta_p_b = - step_size * mapped_gradients_b
print("delta_p Gaussian: {}".format(delta_p_a))
print("delta_p RiemannSum: {}".format(delta_p_b))

delta_z_a = A.MappingMatrix @ delta_p_a
delta_z_b = B.MappingMatrix @ delta_p_b
print("delta_z Gaussian: {}".format(delta_z_a))
print("delta_z RiemannSum: {}".format(delta_z_b))
