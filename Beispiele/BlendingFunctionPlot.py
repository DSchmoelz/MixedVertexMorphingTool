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
from plots import *
# import matplotlib.pyplot as plt
# from matplotlib.ticker import MaxNLocator
# plt.style.use('seaborn-v0_8-paper')
# SMALL_SIZE = 8
# MEDIUM_SIZE = 10
# BIGGER_SIZE = 12

# plt.rc('font', size=BIGGER_SIZE)          # controls default text sizes
# plt.rc('axes', titlesize=BIGGER_SIZE)     # fontsize of the axes title
# plt.rc('axes', labelsize=BIGGER_SIZE)    # fontsize of the x and y labels
# plt.rc('xtick', labelsize=MEDIUM_SIZE)    # fontsize of the tick labels
# plt.rc('ytick', labelsize=MEDIUM_SIZE)    # fontsize of the tick labels
# plt.rc('legend', fontsize=BIGGER_SIZE)    # legend fontsize
# plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

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

shape_functions_vm = VM_param.MappingMatrix * 4 * nodes_per_x
shape_functions_rb = RB_param.MappingMatrix @ RB_param.scaling_matrix

blended_shape_functions_vm = VM_param.MappingMatrix * blending_vm[:, np.newaxis] * 4 * nodes_per_x
blended_shape_functions_vm2 = VM_param.MappingMatrix * blending_vm2[:, np.newaxis] * 4 * nodes_per_x
blended_shape_functions_rb = (RB_param.MappingMatrix @ RB_param.scaling_matrix) * blending_rb[:, np.newaxis]
# Mapping_Matrix = Parameterization.MappingMatrix @ Parameterization.scaling_matrix
# blended_shape_functions_vm = Parameterization.MappingMatrix[:,:-2] * blending_vm[:, np.newaxis]
# blended_shape_functions_rb = Parameterization.MappingMatrix[:,-2:] * blending_rb[:, np.newaxis]

transition = np.arange(blending_vm_x_min_max[1], blending_rb_x_min_max[0]+1)
vm_1 = np.arange(blending_vm_x_min_max[0], blending_vm_x_min_max[1]+1)
rb = np.arange(blending_rb_x_min_max[0], blending_rb_x_min_max[1]-length_vm2+1)
rb_vm_2 = np.arange(blending_vm2_x_min_max[0], blending_vm2_x_min_max[1]+1)
number_of_vm_shape_functions = int(np.shape(blended_shape_functions_vm)[1] / nodes_per_x)

line_style = dict(linewidth=0.5)
marker_style = dict(marker='|', markersize=4)

plot_style = dict(width=3.33, ratio=8/3)

### Morphing functions
plot = Plot(**plot_style)
vlines = [0, 14, 18, 23, 27]
for vline in vlines:
    plt.axvline(vline,
                color=TUM_GRAY_2,
                **line_style)

# functions
# number_of_vm_morphing_functions = int(18)
for i in range(0,18,2):
    node_index = nodes_per_x*i+4*nodes_per_x
    plot.ax.plot(DesignMesh.GetNodeCoordinatesX()[:18*nodes_per_x+1],
                 shape_functions_vm[:18*nodes_per_x+1, nodes_per_x*i],
                 color=TUM_BLUE, **line_style)

for i in range(25,34,2):
    node_index = nodes_per_x*i+4*nodes_per_x
    plot.ax.plot(DesignMesh.GetNodeCoordinatesX()[23*nodes_per_x:32*nodes_per_x+1],
                 shape_functions_vm[23*nodes_per_x:32*nodes_per_x+1, nodes_per_x*i],
                 color=TUM_BLUE_5, **line_style)

plot.ax.plot(DesignMesh.GetNodeCoordinatesX()[14*nodes_per_x:],
             shape_functions_rb[14*nodes_per_x:, 0],
             color=TUM_ORANGE, **line_style)
plot.ax.plot(DesignMesh.GetNodeCoordinatesX()[14*nodes_per_x:],
             shape_functions_rb[14*nodes_per_x:, 1],
             color=TUM_ORANGE, **line_style)


# discretization
plot.ax.plot(np.arange(0,33,1), np.zeros((np.arange(0,33,1)).size),
             color=TUM_GRAY, **line_style,
             **marker_style)
# plot.ax.plot(transition, np.zeros(transition.size),
#              color=TUM_GRAY, label="transition",
#              linewidth=0.75,
#              **marker_style)
# plot.ax.plot(vm_1, np.zeros(vm_1.size),
#              color=TUM_BLUE, label="vm",
#              linewidth=0.75,
#              **marker_style)
# plot.ax.plot(rb, np.zeros(rb.size),
#              color=TUM_ORANGE, label="rb",
#              linewidth=0.75,
#              **marker_style)
# plot.ax.plot(rb_vm_2, np.zeros(rb_vm_2.size),
#              color=TUM_BLUE_5, label="rb+vm",
#              linewidth=0.75,
#              **marker_style)

plot.ax.set_xlim(10-0.05, 28+2+0.05)
plot.ax.set_ylim(-0.7,1.1)
plot.ax.set_xticks([])
plot.ax.set_yticks([])
plt.axis('off')
plt.tight_layout(pad=0.2)
plt.savefig(f"figx_morphing_fcts.pdf")


### Blending functions
plot = Plot(**plot_style)
vlines = [0, 14, 18, 23, 27]
for vline in vlines:
    plt.axvline(vline,
                color=TUM_GRAY_2,
                **line_style)

plot.ax.plot(DesignMesh.GetNodeCoordinatesX(),
             blending_vm[:, np.newaxis],
             color=TUM_BLUE, **line_style)

plot.ax.plot(DesignMesh.GetNodeCoordinatesX(),
             blending_rb[:, np.newaxis],
             color=TUM_ORANGE, **line_style)

plot.ax.plot(DesignMesh.GetNodeCoordinatesX(),
             blending_vm2[:, np.newaxis],
             color=TUM_BLUE_5, **line_style)

# discretization
plot.ax.plot(np.arange(0,33), np.zeros(33),
             color=TUM_GRAY, **line_style,
             **marker_style)
# plot.ax.plot(transition, np.zeros(transition.size),
#              color=TUM_GRAY, label="transition",
#              linewidth=0.75,
#              **marker_style)
# plot.ax.plot(vm_1, np.zeros(vm_1.size),
#              color=TUM_BLUE, label="vm",
#              linewidth=0.75,
#              **marker_style)
# plot.ax.plot(rb, np.zeros(rb.size),
#              color=TUM_ORANGE, label="rb",
#              linewidth=0.75,
#              **marker_style)
# plot.ax.plot(rb_vm_2, np.zeros(rb_vm_2.size),
#              color=TUM_BLUE_5, label="rb+vm",
#              linewidth=0.75,
#              **marker_style)

# plot.ax.set_xlim(8-0.05, 28+4+0.05)
plot.ax.set_xlim(10-0.05, 28+2+0.05)
plot.ax.set_ylim(-0.7,1.1)
plot.ax.set_xticks([])
plot.ax.set_yticks([])
plt.axis('off')
plt.tight_layout(pad=0.2)
plt.savefig(f"figx_blending_fcts.pdf")

### Blended Morphing functions
plot = Plot(**plot_style)
vlines = [0, 14, 18, 23, 27]
for vline in vlines:
    plt.axvline(vline,
                color=TUM_GRAY_2,
                **line_style)

# int((np.shape(blended_shape_functions_vm)[1] - length_of_rb*nodes_per_x - 4*nodes_per_x) / nodes_per_x)
color_map_values = np.linspace(0.2, 0.8, num=number_of_vm_shape_functions)
color_map = matplotlib.cm.get_cmap('Greys')


for i in range(0,18,2):
    node_index = nodes_per_x*i+4*nodes_per_x
    plot.ax.plot(DesignMesh.GetNodeCoordinatesX()[:18*nodes_per_x+1],
                 blended_shape_functions_vm[:18*nodes_per_x+1, nodes_per_x*i],
                 color=TUM_BLUE, **line_style)

for i in range(25,34,2):
    node_index = nodes_per_x*i+4*nodes_per_x
    plot.ax.plot(DesignMesh.GetNodeCoordinatesX()[23*nodes_per_x:32*nodes_per_x+1],
                 blended_shape_functions_vm2[23*nodes_per_x:32*nodes_per_x+1, nodes_per_x*i],
                 color=TUM_BLUE_5, **line_style)

# for i in range(0,number_of_vm_shape_functions,2):
#     node_index = nodes_per_x*i+4*nodes_per_x
#     # axis.plot(DesignMesh.GetNodeCoordinatesX(), blended_shape_functions_vm[:, nodes_per_x*i+4*nodes_per_x], color=color_map(color_map_values[i]), linestyle='-', **line_style)
#     # plot.ax.plot(DesignMesh.GetNodeCoordinatesX(), blended_shape_functions_vm[:, nodes_per_x*i], color=color_map(color_map_values[i]), linestyle='-', **line_style)
#     # plot.ax.plot(DesignMesh.GetNodeCoordinatesX(), blended_shape_functions_vm2[:, nodes_per_x*i], color=color_map(color_map_values[i]), linestyle='-', **line_style)
#     plot.ax.plot(DesignMesh.GetNodeCoordinatesX(),
#                  blended_shape_functions_vm[:, nodes_per_x*i],
#                  color=TUM_BLUE_5, **line_style)
#     plot.ax.plot(DesignMesh.GetNodeCoordinatesX(),
#                  blended_shape_functions_vm2[:, nodes_per_x*i],
#                  color=TUM_BLUE_5, **line_style)

# number_of_vm2_shape_functions = int(np.shape(blended_shape_functions_vm2)[1])  / nodes_per_x
# for i in range(number_of_vm2_shape_functions):
    # plot.ax.plot(DesignMesh.GetNodeCoordinatesX(), blended_shape_functions_vm2[:, i], linestyle='-', **line_style)

plot.ax.plot(DesignMesh.GetNodeCoordinatesX(),
             blended_shape_functions_rb[:, 0],
             color=TUM_ORANGE, **line_style)
plot.ax.plot(DesignMesh.GetNodeCoordinatesX(),
             blended_shape_functions_rb[:, 1],
             color=TUM_ORANGE, **line_style)


# discretization
plot.ax.plot(np.arange(0,33), np.zeros(33),
             color=TUM_GRAY, **line_style,
             **marker_style)
# plot.ax.plot(transition, np.zeros(transition.size),
#              color=TUM_GRAY, label="transition",
#              linewidth=0.75,
#              **marker_style)
# plot.ax.plot(vm_1, np.zeros(vm_1.size),
#              color=TUM_BLUE, label="vm",
#              linewidth=0.75,
#              **marker_style)
# plot.ax.plot(rb, np.zeros(rb.size),
#              color=TUM_ORANGE, label="rb",
#              linewidth=0.75,
#              **marker_style)
# plot.ax.plot(rb_vm_2, np.zeros(rb_vm_2.size),
#              color=TUM_BLUE_5, label="rb+vm",
#              linewidth=0.75,
#              **marker_style)

# plot.ax.set_xticks([0, 14, 18, 23, 27])
# plt.axvline(vlines)
# plot.ax.set_xlim(8-0.05, 28+4+0.05)
plot.ax.set_xlim(10-0.05, 28+2+0.05)
plot.ax.set_ylim(-0.7,1.1)
plot.ax.set_xticks([])
plot.ax.set_yticks([])
plt.axis('off')
plt.tight_layout(pad=0.2)
plt.savefig(f"figx_blended_morphing_fcts.pdf")

# ## Plot
# figure, axis = plt.subplots(figsize=[24.0,5.0])

# line_style = dict(linewidth=1.5)
# style_shape = dict(linewidth=1.5)

# number_of_vm_shape_functions = int(np.shape(blended_shape_functions_vm)[1] / nodes_per_x)
# # int((np.shape(blended_shape_functions_vm)[1] - length_of_rb*nodes_per_x - 4*nodes_per_x) / nodes_per_x)
# color_map_values = np.linspace(0.2, 0.8, num=number_of_vm_shape_functions)
# color_map = matplotlib.cm.get_cmap('Blues')

# for i in range(number_of_vm_shape_functions):
#     node_index = nodes_per_x*i+4*nodes_per_x
#     # axis.plot(DesignMesh.GetNodeCoordinatesX(), blended_shape_functions_vm[:, nodes_per_x*i+4*nodes_per_x], color=color_map(color_map_values[i]), linestyle='-', **line_style)
#     axis.plot(DesignMesh.GetNodeCoordinatesX(), blended_shape_functions_vm[:, nodes_per_x*i], color=color_map(color_map_values[i]), linestyle='-', **line_style)
#     axis.plot(DesignMesh.GetNodeCoordinatesX(), blended_shape_functions_vm2[:, nodes_per_x*i], color=color_map(color_map_values[i]), linestyle='-', **line_style)

# # number_of_vm2_shape_functions = int(np.shape(blended_shape_functions_vm2)[1])  / nodes_per_x
# # for i in range(number_of_vm2_shape_functions):
#     axis.plot(DesignMesh.GetNodeCoordinatesX(), blended_shape_functions_vm2[:, i], linestyle='-', **line_style)


# axis.plot(DesignMesh.GetNodeCoordinatesX(), blended_shape_functions_rb[:, 0], color='darkorange', linestyle='-', **line_style)
# axis.plot(DesignMesh.GetNodeCoordinatesX(), blended_shape_functions_rb[:, 1], color='orange', linestyle='-', **line_style)

# axis.plot(DesignMesh.GetNodeCoordinatesX(), DesignMesh.GetShapeZ(), color='black', **style_shape)

# # axis.plot(DesignMesh.GetNodeCoordinatesX(), DesignMesh.GetShapeZ(), color='lightskyblue', marker='o', markersize=5.0, markerfacecolor='lightskyblue', label='initial', **style_shape)

# axis.plot(rb_rotation_center, DesignMesh.GetGeometryAt(rb_rotation_center), marker='o', markersize=10.0, markerfacecolor='lightsteelblue')

# axis.axvline(x=blending_vm_x_min_max[1], color='grey',linestyle='-')
# axis.axvline(x=blending_rb_x_min_max[0], color='grey',linestyle='-')
# axis.axvline(x=blending_vm2_x_min_max[0]-blending_filter, color='grey',linestyle='-')
# axis.axvline(x=blending_vm2_x_min_max[0], color='grey',linestyle='-')
# axis.set_xlim(8, 28+4)

# figure.savefig("Plots/Blending/blended_shape_functions.png", dpi=600)
# plt.axis('off')
# axis.set_axis_off()

# plt.show()