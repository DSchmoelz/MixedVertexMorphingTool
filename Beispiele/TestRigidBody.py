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
from RigidBodyObjectivePlot import create_objective_plot

def target_geometry(x_j):
    p_j = x_j / 2 + 4

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
scaling_types =["none", "column", "shape"]

fig, ax = create_objective_plot()

for scaling_type in scaling_types:

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
        "scaling": scaling_type
    }
    Mapper = RigidBodyParameterization(DesignMesh, rigid_body_settings)

    StepSizeSettings = ConstStepInUnscaledControl(0.05, Mapper)
    # StepSizeSettings = ConstStepInControl(1.0)

    max_steps = 80
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

    ax.plot(translation, rotation, f, label=scaling_type)
    # ax.plot(translation, rotation, f, label=scaling_type, marker='o')
    ax.legend(title="scaling types")
    # # Plot
    # figure, axis = plt.subplots(2, figsize=[5.0,8.0])
    # axis[0].plot(x_j, p_j, '-*', color='lightgrey', label='target shape')

    # FinalShape = OptimizationAlgorithm.Mapper.Design
    # axis[0].plot(FinalShape.GetNodeCoordinatesX(), FinalShape.GetShapeZ(), '-', label='design shape after {} iterations'.format(ConvergenceSettings.MaxSteps))
    # axis[0].axis('equal')
    # axis[0].legend()

    # axis[1].plot(f)
    # axis[1].set(xlabel="Step", ylabel="Objective")
    # figure.show()

plt.show()
