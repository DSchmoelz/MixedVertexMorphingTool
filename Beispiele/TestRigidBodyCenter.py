#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# TestRigidBody
#####################################################################

import numpy as np
import sys
import path_setting
sys.path.append(path_setting.path)
from vmtool import *
import matplotlib.pyplot as plt
plt.style.use('seaborn')
from RigidBodyObjectivePlot import create_objective_plot

def target_geometry(x_j):
    p_j = x_j / 2 + 8

    return p_j

## Target Geometry
filter_radius = 4
x_limit = filter_radius + 4
target_number_of_nodes = 3
x_j = np.linspace(-x_limit, x_limit, target_number_of_nodes)
p_j = np.zeros(target_number_of_nodes)

TargetNodeList = []
target_ids = np.arange(target_number_of_nodes)
for i in range(0, target_number_of_nodes):
    p_j[i] = target_geometry(x_j[i])
    TargetNodeList.append(ControlNode(target_ids[i], x_j[i], p_j[i]))

TargetMesh = Mesh("target")
TargetMesh.AddNodes(TargetNodeList)

# scaling_types = ["none"]
scaling_types =["none", "column", "shape", "shape_diag_mass", "sens_shape", "sens_shape_diag_mass"]
colors = ['gray', 'green', 'red', 'orange', 'blue', 'cyan']
markers = ['X', 'P', 'o', 's', 'v', '<']
linestyles = ['solid', 'solid', 'solid', 'solid', (0, (5, 10)), (0, (5, 10))]
style = dict(linewidth=1.0)

obj_plot_settings = {
    "center": -8,
    "translation_interval": [-2, 8],
    "rotation_interval": [-0.4, 1.4]
}
fig, ax = create_objective_plot(obj_plot_settings, target_geometry)
figure_2D, axis_2D = plt.subplots(2, 2, figsize=[16.0,16.0])

for scaling_type, color, marker, linestyle in zip(scaling_types, colors, markers, linestyles):

    ## Design Geometry
    design_number_of_nodes = 2*(x_limit)+1
    x_i = np.linspace(-x_limit, x_limit, design_number_of_nodes)
    DesignNodeList = []
    design_ids = np.arange(design_number_of_nodes)
    for i in range(0, design_number_of_nodes):
        DesignNodeList.append(DesignNode(design_ids[i], x_i[i], 0))

    DesignMesh = Mesh("design")
    DesignMesh.AddNodes(DesignNodeList)

    ## Optimization Set-Up
    Response = TargetGeometryResponse("target", DesignMesh, TargetMesh)

    rigid_body_settings = {
        "translation": True,
        "rotation": True,
        "scaling": scaling_type,
        "center": -8
    }
    Mapper = RigidBodyParameterization(DesignMesh, rigid_body_settings)

    # StepSizeSettings = ConstStepInUnscaledControl(0.5, Mapper)
    StepSizeSettings = GoldenSectionLineSearch(100.0, 1e-8, Mapper)
    # StepSizeSettings = ConstStepInControl(1.0)

    max_steps = 10
    ConvergenceSettings = MaxSteps(max_steps)

    OptimizationAlgorithm = SteepestDescentAlgorithm("Optimierung", Mapper, ConvergenceSettings, StepSizeSettings, NormalizeObjGrad=False)
    OptimizationAlgorithm.AddObjective(Response)

    ## Start Optimization
    OptimizationAlgorithm.StartOptimization()
    # print(OptimizationAlgorithm.Mapper.MappingMatrix)
    for i in range(len(OptimizationAlgorithm.PreviousControlFields)):
        print(20*"-")
        print("optimization step {}".format(i+1))
        print("gradient {}".format(OptimizationAlgorithm.PreviousControlFields[i]["dg/dp"]))
        print("control update {}".format(OptimizationAlgorithm.PreviousControlFields[i]["delta_p"]))
        print("objective value {}".format(OptimizationAlgorithm.PreviousObjectiveValue[i]))

    print(40*"-")
    print("final objective value {}".format(OptimizationAlgorithm.PreviousObjectiveValue[-1]))

    f = OptimizationAlgorithm.PreviousObjectiveValue
    print(int(Mapper.ControlSize))
    translation = [0]
    rotation = [0]
    for i in range(max_steps):
        translation.append(OptimizationAlgorithm.ControlParameter[2*i])
        rotation.append(OptimizationAlgorithm.ControlParameter[2*i+1])

    # ax.plot(translation, rotation, f, label=scaling_type)
    ax.plot(translation, rotation, f, color=color, label=scaling_type, linestyle=linestyle)
    ax.legend(title="scaling types")

    # Plot
    # axis[0,0].plot(FinalShape.GetNodeCoordinatesX(), FinalShape.GetShapeZ(), '-', label='shape after {} iterations - scaling type: {}'.format(ConvergenceSettings.MaxSteps, scaling_type))
    # axis[0,0].axis('equal')
    # axis[0,0].legend()

    FinalShape = OptimizationAlgorithm.Mapper.Design
    axis_2D[0,1].plot(translation, color=color, label=scaling_type, marker=marker, linestyle=linestyle, **style)
    axis_2D[0,1].set(xlabel="Step", ylabel="Translation ")
    # axis_2D[1].legend(title="scaling types")
    # axis_2D[1].set_yscale('log')

    axis_2D[1,1].plot(rotation, color=color, marker=marker, linestyle=linestyle, **style)
    axis_2D[1,1].set(xlabel="Step", ylabel="Rotation")
    # axis_2D[2].legend(title="scaling types")
    # axis_2D[2].set_yscale('log')

    axis_2D[0,0].plot(f, color=color, label=scaling_type, marker=marker, linestyle=linestyle, **style)
    axis_2D[0,0].set(xlabel="Step", ylabel="Objective")
    axis_2D[0,0].legend(title="scaling types")

    axis_2D[1,0].plot(f, color=color, label=scaling_type, marker=marker, linestyle=linestyle, **style)
    axis_2D[1,0].set(xlabel="Step", ylabel="Objective")
    axis_2D[1,0].legend(title="scaling types")
    axis_2D[1,0].set_yscale('log')

plt.show()
