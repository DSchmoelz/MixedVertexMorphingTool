#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# TestRigidBody
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

def target_geometry(x_j, filter_radius, blending_rb_x_min_max, blending_vm_x_min_max):
    rotation_center = (blending_rb_x_min_max[1] - blending_rb_x_min_max[0]) / 2 + blending_rb_x_min_max[0]
    rb_length = (blending_rb_x_min_max[1] - blending_rb_x_min_max[0])
    vm_length = blending_vm_x_min_max[1] - blending_vm_x_min_max[0]

    length_of_cos = vm_length
    amplitude = 2
    gradient_at_end = amplitude*1.5*np.pi/length_of_cos
    if x_j <= filter_radius:
        p_j = 0
    elif x_j > filter_radius and x_j <= filter_radius+vm_length:

        p_j = amplitude + amplitude*np.cos((x_j-filter_radius) * (1.5*np.pi/length_of_cos) + np.pi )
    else:
        p_j = -gradient_at_end*(x_j-rotation_center) - (gradient_at_end*rb_length/2-amplitude)
    return p_j

filter_radius = 4
blending_filter = 4
x_limit = 4+24
blending_vm_x_min_max = [0, int(4+(x_limit-4)/2-blending_filter/2)]
blending_rb_x_min_max = [int(4+(x_limit-4)/2+blending_filter/2), x_limit]
rb_rotation_center = (blending_rb_x_min_max[1] - blending_rb_x_min_max[0]) / 2 + blending_rb_x_min_max[0]
# print("blending_vm_x_min_max: {}".format(blending_vm_x_min_max))
# print("blending_rb_x_min_max: {}".format(blending_rb_x_min_max))
# print("rb_rotation_center: {}".format(rb_rotation_center))

## Target Geometry
nodes_per_x = 1
number_of_nodes = nodes_per_x*(x_limit)+1
x_i = np.linspace(0, x_limit, number_of_nodes)
p_i = np.zeros(number_of_nodes)

TargetNodeList = []
target_ids = np.arange(number_of_nodes)
for i in range(0, number_of_nodes):
    TargetNodeList.append(ControlNode(target_ids[i], x_i[i], 0))

TargetMesh = Mesh("target")
TargetMesh.AddNodes(TargetNodeList)

## Compute VM Blending
blending_vm_nodes = []
blending_rb_nodes = []
for node in TargetMesh.Nodes:
    if node.x >= blending_vm_x_min_max[0] and node.x <= blending_vm_x_min_max[1]:
        blending_vm_nodes.append(node)
    if node.x >= blending_rb_x_min_max[0] and node.x <= blending_rb_x_min_max[1]:
        blending_rb_nodes.append(node)

blending_vm = TargetMesh.ComputeBlendingFunction(blending_vm_nodes, blending_filter)
blending_rb = TargetMesh.ComputeBlendingFunction(blending_rb_nodes, blending_filter)

for node in TargetMesh.Nodes:
    index = TargetMesh.GetNodeIndex(node.id)
    node.z = target_geometry(node.x, blending_filter,
                             blending_rb_x_min_max, blending_vm_x_min_max)

# all scaling types
# scaling_types =["none", "column", "shape", "shape_diag_mass", "sens_shape", "sens_shape_diag_mass"]
# colors = ['gray', 'green', 'red', 'orange', 'blue', 'cyan']
# markers = ['X', 'P', 'o', 's', 'v', '<']
# linestyles = ['solid', 'solid', 'solid', 'solid', (0, (5, 10)), (0, (5, 10))]

# all scaling types beside pure sensitivity scaling
# scaling_types =["none", "column", "shape", "shape_diag_mass"]
# colors = ['gray', 'green', 'red', 'orange']
# markers = ['X', 'P', 'o', 's']
# plot_steps = [5, 1, 1, 3]
# linestyles = ['solid', 'solid', 'solid', 'solid']

# no scaling at all
# scaling_types = ["none"]
# colors = ['gray']
# markers = ['X']
# plot_steps = [5]
# linestyles = ['solid']

# Paper scaling types
scaling_types =["shape_diag", "shape", "shape_w_off"]
colors = ['gray', 'red', 'orange']
markers = ['X', 'o', 's']
number_of_plots = [4, 2, 2]
linestyles = ['solid', 'solid', 'solid']

# scaling_types =["none", "shape"]
# colors = ['gray', 'red', ]
# markers = ['X', 'o']
# number_of_plots = [4, 2]
# linestyles = ['solid', 'solid']

# # Paper ohne none scaling
# scaling_types =["shape", "shape_w_off"]
# colors = ['red', 'orange']
# markers = ['o', 's']
# number_of_plots = [2, 2]
# linestyles = ['solid', 'solid']

style = dict(linewidth=1.0)

figure_2D, axis_2D = plt.subplots(2, 2, figsize=[12.0,8.0])
figure_2D.tight_layout(pad=2.0)

figure_conv_logx, axis_conv_logx = plt.subplots(2, 2, figsize=[12.0,8.0])
figure_conv_logx.tight_layout(pad=2.0)

for scaling_type, color, marker, linestyle, plot_number in zip(scaling_types, colors, markers, linestyles, number_of_plots):

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

    figure_shape, axis_shape = plt.subplots(1, figsize=[5.0,5.0])
    axis_shape.plot(TargetMesh.GetNodeCoordinatesX(), TargetMesh.GetShapeZ(), label='target')
    axis_shape.plot(DesignMesh.GetNodeCoordinatesX(), DesignMesh.GetShapeZ(), color='lightskyblue', marker='o', markersize=5.0, markerfacecolor='lightskyblue', label='initial')
    axis_shape.plot(rb_rotation_center, DesignMesh.GetGeometryAt(rb_rotation_center), marker='o', markersize=10.0, markerfacecolor='lightsteelblue')
    axis_shape.set_xlabel(xlabel="local coordinate " +  r'$\xi$')
    axis_shape.axis('equal')
    axis_shape.legend()
    figure_shape.tight_layout()

    figure_shape.savefig("Plots/SideBySide/optproblem.png", dpi=600)

    ## Compute VM Blending
    blending_vm_nodes = []
    blending_rb_nodes = []
    for node in DesignMesh.Nodes:
        if node.x >= blending_vm_x_min_max[0] and node.x <= blending_vm_x_min_max[1]:
            blending_vm_nodes.append(node)
        if node.x >= blending_rb_x_min_max[0] and node.x <= blending_rb_x_min_max[1]:
            blending_rb_nodes.append(node)

    blending_vm = DesignMesh.ComputeBlendingFunction(blending_vm_nodes, blending_filter)
    blending_rb = DesignMesh.ComputeBlendingFunction(blending_rb_nodes, blending_filter)

    ## Optimization Set-Up
    Response = TargetGeometryResponse("target", DesignMesh, TargetMesh)


    if scaling_type in ["shape", "shape_diag"]:
        scaling_sub = "shape"
    else:
        scaling_sub = "none"
    ## Vertex Morphing Parameterization
    vm_settings = {
        "filter_radius": filter_radius,
        "integration": "RiemannSum",
        "scaling": scaling_sub
    }
    VM_param = VertexMorphing(DesignMesh, ControlMesh, vm_settings)

    ## Rigid Body Parameterization
    if scaling_type in ["shape_diag"]:
        scaling_sub = "shape_diag_mass"
    rigid_body_settings = {
        "translation": True,
        "rotation": True,
        "scaling": scaling_sub,
        "center": rb_rotation_center
    }
    RB_param = RigidBodyParameterization(DesignMesh, rigid_body_settings)

    ## Vertex Morphing + Rigid Body
    settings = {
            "scaling": scaling_type
        }
    Parameterization = VertexMorphingRigidBodyParameterization(VM_param, RB_param, settings, VertexMorphingBlending=blending_vm, RigidBodyBlending=blending_rb)

    max_step_size = 5
    line_search_tolerance = 1e-6
    # StepSizeSettings = ConstStepInUnscaledControl(0.5, Parameterization)
    # StepSizeSettings = ConstStepInControl(.1.0)
    if scaling_type == "none":
        StepSizeSettings = GoldenSectionLineSearch(max_step_size, line_search_tolerance, Parameterization)
    else:
        StepSizeSettings = GoldenSectionLineSearch(max_step_size, line_search_tolerance, Parameterization)

    # max_steps = 1500
    max_steps = 10000
    objective_value = 1e-2
    # ConvergenceSettings = MaxSteps(max_steps)
    ConvergenceSettings = ObjectiveValue(objective_value, max_steps)

    history_folder = f"history_scaling_{scaling_type}"
    OptimizationAlgorithm = SteepestDescentAlgorithm("Optimization with scaling type '{}'".format(scaling_type), Parameterization, ConvergenceSettings, StepSizeSettings, NormalizeObjGrad=False, HistoryFolder=history_folder)
    OptimizationAlgorithm.AddObjective(Response)

    ## Start Optimization
    OptimizationAlgorithm.StartOptimization()

    p = np.zeros(Parameterization.ControlSize)
    translation = [0]
    rotation = [0]
    for i in range(len(OptimizationAlgorithm.PreviousControlFields)):
        # print(20*"-")
        # print("optimization step {}".format(i+1))
        control_size = len(OptimizationAlgorithm.PreviousControlFields[i]["delta_p"])
        p = OptimizationAlgorithm.ControlParameter[i*control_size:i*control_size+control_size]
        translation.append(p[-2])
        rotation.append(p[-1])

        # print("gradient {}".format(OptimizationAlgorithm.PreviousControlFields[i]["dg/dp"]))
        # print("control update {}".format(OptimizationAlgorithm.PreviousControlFields[i]["delta_p"]))
        # print("objective value {}".format(OptimizationAlgorithm.PreviousObjectiveValue[i]))

    # print(40*"-")
    # print("final objective value {}".format(OptimizationAlgorithm.PreviousObjectiveValue[-1]))

    f = OptimizationAlgorithm.PreviousObjectiveValue
    # print(f"f: {f}")
    final_step = OptimizationAlgorithm.StepNumber-1

    axis_2D[0,1].plot(translation, color=color, label=scaling_type, linestyle=linestyle, **style)
    axis_2D[0,1].scatter(final_step, translation[-1], color=color, marker=marker)
    axis_2D[0,1].set(xlabel="Iteration", ylabel="Translation ")
    axis_2D[0,1].xaxis.set_major_locator(MaxNLocator(integer=True))

    axis_2D[1,1].plot(rotation, color=color, linestyle=linestyle, **style)
    axis_2D[1,1].scatter(final_step, rotation[-1], color=color, marker=marker)
    axis_2D[1,1].set(xlabel="Iteration", ylabel="Rotation")
    axis_2D[1,1].xaxis.set_major_locator(MaxNLocator(integer=True))

    axis_2D[0,0].plot(f, color=color, label=scaling_type, linestyle=linestyle, **style)
    axis_2D[0,0].scatter(final_step, f[-1], color=color, marker=marker)
    axis_2D[0,0].set(xlabel="Iteration", ylabel="Objective")
    axis_2D[0,0].xaxis.set_major_locator(MaxNLocator(integer=True))
    axis_2D[0,0].legend(title="scaling types")

    axis_2D[1,0].plot(f, color=color, label=scaling_type, linestyle=linestyle, **style)
    axis_2D[1,0].scatter(final_step, f[-1], color=color, marker=marker)
    axis_2D[1,0].set(xlabel="Iteration", ylabel="Objective")
    axis_2D[1,0].xaxis.set_major_locator(MaxNLocator(integer=True))
    axis_2D[1,0].set_yscale('log')
    axis_2D[1,0].axhline(y=objective_value, color='magenta',linestyle=linestyle, **style)

    # Convergence plot mit log x achs
    axis_conv_logx[0,1].plot(translation, color=color, label=scaling_type, linestyle=linestyle, **style)
    axis_conv_logx[0,1].scatter(final_step, translation[-1], color=color, marker=marker)
    axis_conv_logx[0,1].set(xlabel="Iteration", ylabel="Translation ")
    axis_conv_logx[0,1].xaxis.set_major_locator(MaxNLocator(integer=True))
    axis_conv_logx[0,1].set_xscale('symlog')

    axis_conv_logx[1,1].plot(rotation, color=color, linestyle=linestyle, **style)
    axis_conv_logx[1,1].scatter(final_step, rotation[-1], color=color, marker=marker)
    axis_conv_logx[1,1].set(xlabel="Iteration", ylabel="Rotation")
    axis_conv_logx[1,1].xaxis.set_major_locator(MaxNLocator(integer=True))
    axis_conv_logx[1,1].set_xscale('symlog')

    axis_conv_logx[0,0].plot(f, color=color, label=scaling_type, linestyle=linestyle, **style)
    axis_conv_logx[0,0].scatter(final_step, f[-1], color=color, marker=marker)
    axis_conv_logx[0,0].set(xlabel="Iteration", ylabel="Objective")
    axis_conv_logx[0,0].xaxis.set_major_locator(MaxNLocator(integer=True))
    axis_conv_logx[0,0].legend(title="scaling types")
    axis_conv_logx[0,0].set_xscale('symlog')

    axis_conv_logx[1,0].plot(f, color=color, label=scaling_type, linestyle=linestyle, **style)
    axis_conv_logx[1,0].scatter(final_step, f[-1], color=color, marker=marker)
    axis_conv_logx[1,0].set(xlabel="Iteration", ylabel="Objective")
    axis_conv_logx[1,0].xaxis.set_major_locator(MaxNLocator(integer=True))
    axis_conv_logx[1,0].set_xscale('symlog')
    axis_conv_logx[1,0].set_yscale('log')
    axis_conv_logx[1,0].axhline(y=objective_value, color='magenta',linestyle=linestyle, **style)

    color_map_values = np.linspace(0.3, 0.8, num=plot_number)
    plot_steps = np.geomspace(1, final_step, num=plot_number, dtype=int, endpoint=False)
    color_map = matplotlib.cm.get_cmap('Greys')
    for i in range(plot_number):
        x = OptimizationAlgorithm.PreviousDesignFields[plot_steps[i]]["x"]
        z = OptimizationAlgorithm.PreviousDesignFields[plot_steps[i]]["z"]
        axis_shape.plot(x, z, color=color_map(color_map_values[i]), marker='o', markersize=5, label="iteration {}".format(plot_steps[i]))

    FinalShape = OptimizationAlgorithm.Mapper.Design
    axis_shape.plot(FinalShape.GetNodeCoordinatesX(), FinalShape.GetShapeZ(), color='black', marker='o', markersize=5, label="iteration {}".format(final_step))
    axis_shape.axis('equal')
    axis_shape.legend()
    figure_shape.savefig("Plots/SideBySide/shape_{}.png".format(scaling_type), dpi=600)

figure_2D.savefig("Plots/SideBySide/convergence_plot.png", dpi=600)
figure_conv_logx.savefig("Plots/SideBySide/convergence_plot_logx.png", dpi=600)
plt.show()
