#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# TestVMRigidBody
#####################################################################

import numpy as np
import sys
import path_setting
sys.path.append(path_setting.path)
from vmtool import *
import matplotlib
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

def target_geometry(x_j, blending_x_min_max):
    if x_j >= -blending_x_min_max and x_j <= blending_x_min_max:
        p_j = x_j / 2 + 4 + (np.cos((x_j+blending_x_min_max)*np.pi/blending_x_min_max) - 1)
    else:
        p_j = x_j / 2 + 4
    # p_j = x_j / 2 + 4 + (x_j**2)/10

    return p_j

blending_x_min_max = 4
filter_radius = 2
## Target Geometry
x_limit = 12
nodes_per_x = 2


# blending_x_min_max = 1
# filter_radius = 2
# # Target Geometry
# x_limit = 3
# nodes_per_x = 1

# x_limit_target = filter_radius + 8
# x_limit = filter_radius
number_of_nodes = 2*nodes_per_x*(x_limit)+1
# target_number_of_nodes = 8*(x_limit)+1
x_i = np.linspace(-x_limit, x_limit, number_of_nodes)
p_i = np.zeros(number_of_nodes)

TargetNodeList = []
target_ids = np.arange(number_of_nodes)
for i in range(0, number_of_nodes):
    p_i[i] = target_geometry(x_i[i], blending_x_min_max)
    TargetNodeList.append(ControlNode(target_ids[i], x_i[i], p_i[i]))

TargetMesh = Mesh("target")
TargetMesh.AddNodes(TargetNodeList)

# scaling_types =["none", "shape", "shape_w_off"]
# colors = ['gray', 'red', 'orange']
# markers = ['X', 'o', 's']
# number_of_plots = [4, 4, 2]
# linestyles = ['solid', 'solid', 'solid']

# Paper scaling types
# scaling_types =["none",  "shape_diag", "shape", "shape_w_off"]
# colors = ['gray', "#a2ad00", 'red', 'orange']
# markers = ['X', '2', 'o', 's']
# number_of_plots = [4, 4, 4, 2]
# linestyles = ['solid', 'solid', 'solid', 'solid']

scaling_types =["shape_diag", "shape", "shape_w_off"]
colors = ["#a2ad00", 'red', 'orange']
markers = ['2', 'o', 's']
number_of_plots = [4, 4, 2]
linestyles = ['solid', 'solid', 'solid']

style = dict(linewidth=1.0)
figure_2D, axis_2D = plt.subplots(2, 2, figsize=[12.0,8.0])
figure_2D.tight_layout(pad=2.0)

figure_conv_logx, axis_conv_logx = plt.subplots(2, 2, figsize=[12.0,8.0])
figure_conv_logx.tight_layout(pad=2.0)

for scaling_type, color, marker, linestyle, plot_number in zip(scaling_types, colors, markers, linestyles, number_of_plots):

    ## Control Geometry
    ControlNodeList = []
    control_ids = np.arange(number_of_nodes)
    # c_i = np.zeros(number_of_nodes)
    for i in range(0, number_of_nodes):
        ControlNodeList.append(ControlNode(control_ids[i], x_i[i], 0))

    ControlMesh = Mesh("control")
    ControlMesh.AddNodes(ControlNodeList)

    ## Design Geometry
    DesignNodeList = []
    design_ids = np.arange(number_of_nodes)
    for i in range(0, number_of_nodes):
        DesignNodeList.append(DesignNode(design_ids[i], x_i[i], 0))

    DesignMesh = Mesh("design")
    DesignMesh.AddNodes(DesignNodeList)

    ## Compute VM Blending
    blending_node_ids = []
    for node in DesignMesh.Nodes:
        if node.x >= -blending_x_min_max and node.x <= blending_x_min_max:
            blending_node_ids.append(node)

    vm_blending_function = DesignMesh.ComputeBlendingFunction(blending_node_ids, filter_radius)

    figure_shape, axis_shape = plt.subplots(1, figsize=[5.0,5.0])
    axis_shape.plot(TargetMesh.GetNodeCoordinatesX(), TargetMesh.GetShapeZ(), label='target')
    axis_shape.plot(DesignMesh.GetNodeCoordinatesX(), DesignMesh.GetShapeZ(), color='lightskyblue', marker='o', markersize=5.0, markerfacecolor='lightskyblue', label='initial')
    axis_shape.plot(0, DesignMesh.GetGeometryAt(0), marker='o', markersize=10.0, markerfacecolor='lightsteelblue')
    axis_shape.set_xlabel(xlabel="local coordinate " +  r'$\xi$')
    axis_shape.axis('equal')
    axis_shape.legend()
    figure_shape.tight_layout()

    figure_shape.savefig("Plots/Nested/optproblem.png", dpi=600)

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
        "scaling": scaling_sub
    }
    RB_param = RigidBodyParameterization(DesignMesh, rigid_body_settings)

    ## Vertex Morphing + Rigid Body
    settings = {
            "scaling": scaling_type
        }
    # if scaling_type == "shape_sub":
    #     scaling_type = "shape"
    Parameterization = VertexMorphingRigidBodyParameterization(VM_param, RB_param, settings, VertexMorphingBlending=vm_blending_function)

    step_size = 0.1
    max_step_size = 5
    line_search_tolerance = 1e-6
    # StepSizeSettings = ConstStepInUnscaledControl(0.5, Parameterization)
    # StepSizeSettings = ConstStepInControl(step_size)
    if scaling_type == "none":
        StepSizeSettings = GoldenSectionLineSearch(0.21, line_search_tolerance, Parameterization)
    else:
        StepSizeSettings = GoldenSectionLineSearch(max_step_size, line_search_tolerance, Parameterization)

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
        # print("gradient {}".format(OptimizationAlgorithm.PreviousControlFields[i]["dg/dp"]))
        # print("control update {}".format(OptimizationAlgorithm.PreviousControlFields[i]["delta_p"]))
        control_size = len(OptimizationAlgorithm.PreviousControlFields[i]["delta_p"])
        p = OptimizationAlgorithm.ControlParameter[i*control_size:i*control_size+control_size]
        translation.append(p[-2])
        rotation.append(p[-1])
        # print("control values {}".format(p))
        # print("objective value {}".format(OptimizationAlgorithm.PreviousObjectiveValue[i]))

    # print(40*"-")
    # print("final objective value {}".format(OptimizationAlgorithm.PreviousObjectiveValue[-1]))

    f = OptimizationAlgorithm.PreviousObjectiveValue

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
    plot_steps = np.linspace(1, final_step, num=plot_number, dtype=int, endpoint=False)
    color_map = matplotlib.cm.get_cmap('Greys')
    for i in range(plot_number):
        x = OptimizationAlgorithm.PreviousDesignFields[plot_steps[i]]["x"]
        z = OptimizationAlgorithm.PreviousDesignFields[plot_steps[i]]["z"]
        axis_shape.plot(x, z, color=color_map(color_map_values[i]), marker='o', markersize=5, label="iteration {}".format(plot_steps[i]))

    FinalShape = OptimizationAlgorithm.Mapper.Design
    axis_shape.plot(FinalShape.GetNodeCoordinatesX(), FinalShape.GetShapeZ(), color='black', marker='o', markersize=5, label="iteration {}".format(final_step))
    axis_shape.axis('equal')
    axis_shape.legend()
    figure_shape.savefig("Plots/Nested/shape_{}.png".format(scaling_type), dpi=600)

figure_2D.savefig("Plots/Nested/convergence_plot.png", dpi=600)
figure_conv_logx.savefig("Plots/Nested/convergence_plot_logx.png", dpi=600)
plt.show()


# ### pure VM Optimization

# ## Control Geometry
# # x_limit = filter_radius + 8
# # control_number_of_nodes = 2*(x_limit)+1
# # x_i = np.linspace(-x_limit, x_limit, number_of_nodes)
# ControlNodeList2 = []
# control_ids = np.arange(number_of_nodes)
# # c_j = np.zeros(number_of_nodes)
# for i in range(0, number_of_nodes):
#     ControlNodeList2.append(ControlNode(control_ids[i], x_i[i],0))

# ControlMesh2 = Mesh("control_2")
# ControlMesh2.AddNodes(ControlNodeList)

# ## Design Geometry
# # design_number_of_nodes = 2*(x_limit)+1
# DesignNodeList2 = []
# design_ids = np.arange(number_of_nodes)
# for i in range(0, number_of_nodes):
#     DesignNodeList2.append(DesignNode(design_ids[i], x_i[i], 0))

# DesignMesh2 = Mesh("design_2")
# DesignMesh2.AddNodes(DesignNodeList2)

# ## Optimization Set-Up
# Response2 = TargetGeometryResponse("target", DesignMesh2, TargetMesh)

# ## Vertex Morphing Parameterization
# vm_settings = {
#     "filter_radius": filter_radius,
#     "integration": "RiemannSum",
#     "scaling": "none"
# }
# VM_pure_param = VertexMorphing(DesignMesh2, ControlMesh2, vm_settings)

# # StepSizeSettings2 = ConstStepInUnscaledControl(step_size, VM_pure_param)
# # StepSizeSettings2 = ConstStepInControl(step_size)
# StepSizeSettings2 = GoldenSectionLineSearch(max_step_size, line_search_tolerance, VM_pure_param)

# ConvergenceSettings2 = MaxSteps(max_steps)

# Optimization_VM = SteepestDescentAlgorithm("Optimierung", VM_pure_param, ConvergenceSettings2, StepSizeSettings2, NormalizeObjGrad=False)
# Optimization_VM.AddObjective(Response2)

# ## Start Optimization
# Optimization_VM.StartOptimization()

# f2 = Optimization_VM.PreviousObjectiveValue

# FinalShape2 = Optimization_VM.Mapper.Design
# axis[0,0].plot(FinalShape2.GetNodeCoordinatesX(), FinalShape2.GetShapeZ(), '-', label='design shape after {} iterations - VM'.format(ConvergenceSettings.MaxSteps))
# axis[0,0].axis('equal')
# axis[0,0].legend()

# axis[1,0].plot(f2, label='VM')
# axis[1,0].legend()
# axis[0,1].legend()
# axis[1,1].legend()

# axis[1,0].set_yscale('log')
# plt.show()
