#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Test Blending Functions
#####################################################################
import matplotlib
from pyparsing import line
import numpy as np
import sys
import path_setting
sys.path.append(path_setting.path)
from vmtool import *
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
plt.style.use('seaborn-v0_8-paper')
SMALL_SIZE = 8
MEDIUM_SIZE = 10
BIGGER_SIZE = 12

plt.rc('font', size=BIGGER_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=BIGGER_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=BIGGER_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=MEDIUM_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=MEDIUM_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=BIGGER_SIZE)    # legend fontsize
plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

filter_radius = 4
blending_filter = 4
x_limit = 4+24
length_vm2 = 8

blending_vm_x_min_max = [0, int(4+(x_limit-4)/2-blending_filter/2)]
blending_vm2_x_min_max = [x_limit-1, x_limit+length_vm2]

blending_rb_x_min_max = [int(4+(x_limit-4)/2+blending_filter/2), x_limit+length_vm2]

rb_rotation_center = (blending_rb_x_min_max[1]-length_vm2 - blending_rb_x_min_max[0]) / 2 + blending_rb_x_min_max[0]
length_of_rb = (blending_rb_x_min_max[1] - blending_rb_x_min_max[0])

nodes_per_x = 4
number_of_nodes = nodes_per_x*(x_limit+length_vm2)+1
x_i = np.linspace(0, x_limit+length_vm2, number_of_nodes)

## Control Geometry
ControlNodeList = []
control_ids = np.arange(number_of_nodes)
for i in range(0, number_of_nodes):
    ControlNodeList.append(DesignNode(control_ids[i], x_i[i], 0))

ControlMesh = Mesh("control")
ControlMesh.AddNodes(ControlNodeList)

## Design Geometry
DesignNodeList = []
design_ids = np.arange(number_of_nodes)
for i in range(0, number_of_nodes):
    DesignNodeList.append(DesignNode(design_ids[i], x_i[i], 0))

DesignMesh = Mesh("design")
DesignMesh.AddNodes(DesignNodeList)

## Compute Blendings
blending_vm_nodes = []
blending_vm2_nodes = []
blending_rb_nodes = []
for node in DesignMesh.Nodes:
    if node.x >= blending_vm_x_min_max[0] and node.x <= blending_vm_x_min_max[1]:
        blending_vm_nodes.append(node)
    if node.x >= blending_vm2_x_min_max[0] and node.x <= blending_vm2_x_min_max[1]:
        blending_vm2_nodes.append(node)
    if node.x >= blending_rb_x_min_max[0] and node.x <= blending_rb_x_min_max[1]:
        blending_rb_nodes.append(node)

blending_vm = DesignMesh.ComputeBlendingFunction(blending_vm_nodes, blending_filter)
blending_vm2 = DesignMesh.ComputeBlendingFunction(blending_vm2_nodes, blending_filter)
blending_rb = DesignMesh.ComputeBlendingFunction(blending_rb_nodes, blending_filter)

# scaling_type = "shape_sub"
# if scaling_type in ["shape_sub"]:
#     scaling_sub = "shape"
# else:
#     scaling_sub = "none"

## Vertex Morphing Parameterization
vm_settings = {
    "filter_radius": filter_radius,
    "integration": "RiemannSum",
    "scaling": "none"
}
VM_param = VertexMorphing(DesignMesh, ControlMesh, vm_settings)

## Rigid Body Parameterization
rigid_body_settings = {
    "translation": True,
    "rotation": True,
    "scaling": "column",
    "center": rb_rotation_center
}
RB_param = RigidBodyParameterization(DesignMesh, rigid_body_settings)

# ## Vertex Morphing + Rigid Body
# settings = {
#         "scaling": scaling_type
#     }
# if scaling_type == "shape_sub":
#     scaling_type = "shape"
# Parameterization = VertexMorphingRigidBodyParameterization(VM_param, RB_param, settings, VertexMorphingBlending=blending_vm, RigidBodyBlending=blending_rb)

# Compute Params
VM_param.Calculate()
RB_param.Calculate()
# Parameterization.Calculate()

shape_functions_vm = VM_param.MappingMatrix * blending_vm[:, np.newaxis] * 4 * nodes_per_x
shape_functions_vm2 = VM_param.MappingMatrix * blending_vm2[:, np.newaxis] * 4 * nodes_per_x
shape_functions_rb = (RB_param.MappingMatrix @ RB_param.scaling_matrix) * blending_rb[:, np.newaxis]
# Mapping_Matrix = Parameterization.MappingMatrix @ Parameterization.scaling_matrix
# shape_functions_vm = Parameterization.MappingMatrix[:,:-2] * blending_vm[:, np.newaxis]
# shape_functions_rb = Parameterization.MappingMatrix[:,-2:] * blending_rb[:, np.newaxis]

## Plot
figure, axis = plt.subplots(figsize=[24.0,5.0])

style_blending = dict(linewidth=1.5)
style_shape = dict(linewidth=1.5)

number_of_vm_shape_functions = int(np.shape(shape_functions_vm)[1] / nodes_per_x)
# int((np.shape(shape_functions_vm)[1] - length_of_rb*nodes_per_x - 4*nodes_per_x) / nodes_per_x)
color_map_values = np.linspace(0.2, 0.8, num=number_of_vm_shape_functions)
color_map = matplotlib.cm.get_cmap('Blues')

for i in range(number_of_vm_shape_functions):
    node_index = nodes_per_x*i+4*nodes_per_x
    # axis.plot(DesignMesh.GetNodeCoordinatesX(), shape_functions_vm[:, nodes_per_x*i+4*nodes_per_x], color=color_map(color_map_values[i]), linestyle='-', **style_blending)
    axis.plot(DesignMesh.GetNodeCoordinatesX(), shape_functions_vm[:, nodes_per_x*i], color=color_map(color_map_values[i]), linestyle='-', **style_blending)
    axis.plot(DesignMesh.GetNodeCoordinatesX(), shape_functions_vm2[:, nodes_per_x*i], color=color_map(color_map_values[i]), linestyle='-', **style_blending)

# number_of_vm2_shape_functions = int(np.shape(shape_functions_vm2)[1])  / nodes_per_x
# for i in range(number_of_vm2_shape_functions):
    axis.plot(DesignMesh.GetNodeCoordinatesX(), shape_functions_vm2[:, i], linestyle='-', **style_blending)


axis.plot(DesignMesh.GetNodeCoordinatesX(), shape_functions_rb[:, 0], color='darkorange', linestyle='-', **style_blending)
axis.plot(DesignMesh.GetNodeCoordinatesX(), shape_functions_rb[:, 1], color='orange', linestyle='-', **style_blending)

axis.plot(DesignMesh.GetNodeCoordinatesX(), DesignMesh.GetShapeZ(), color='black', **style_shape)

# axis.plot(DesignMesh.GetNodeCoordinatesX(), DesignMesh.GetShapeZ(), color='lightskyblue', marker='o', markersize=5.0, markerfacecolor='lightskyblue', label='initial', **style_shape)

axis.plot(rb_rotation_center, DesignMesh.GetGeometryAt(rb_rotation_center), marker='o', markersize=10.0, markerfacecolor='lightsteelblue')

axis.axvline(x=blending_vm_x_min_max[1], color='grey',linestyle='-')
axis.axvline(x=blending_rb_x_min_max[0], color='grey',linestyle='-')
axis.axvline(x=blending_vm2_x_min_max[0]-blending_filter, color='grey',linestyle='-')
axis.axvline(x=blending_vm2_x_min_max[0], color='grey',linestyle='-')
axis.set_xlim(8, 28+4)

figure.savefig("Plots/Blending/blended_shape_functions.png", dpi=600)
plt.axis('off')
axis.set_axis_off()

plt.show()