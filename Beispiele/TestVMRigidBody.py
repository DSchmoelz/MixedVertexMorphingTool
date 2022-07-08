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
import matplotlib.pyplot as plt
from RigidBodyObjectivePlot import create_objective_plot

def target_geometry(x_j):
    # if x_j > -4 and x_j < 4:
    #     p_j = x_j / 2 + 4 + (x_j**2)/10
    # else:
    #     p_j = 0
    p_j = x_j / 2 + 4 + (x_j**2)/10

    return p_j

## Target Geometry
filter_radius = 4
x_limit_target = filter_radius + 4
# x_limit = filter_radius
target_number_of_nodes = 2*(x_limit_target)+1
x_j = np.linspace(-x_limit_target, x_limit_target, target_number_of_nodes)
p_j = np.zeros(target_number_of_nodes)

TargetNodeList = []
target_ids = np.arange(target_number_of_nodes)
for i in range(0, target_number_of_nodes):
    p_j[i] = target_geometry(x_j[i])
    TargetNodeList.append(ControlNode(target_ids[i], x_j[i], p_j[i]))

TargetMesh = Mesh("target")
TargetMesh.AddNodes(TargetNodeList)

# scaling_types = ["none"]
scaling_types =["none"]

# fig, ax = create_objective_plot()

for scaling_type in scaling_types:

    ## Control Geometry
    x_limit = filter_radius + 4
    control_number_of_nodes = 2*(x_limit)+1
    x_i = np.linspace(-x_limit, x_limit, control_number_of_nodes)
    ControlNodeList = []
    control_ids = np.arange(control_number_of_nodes)
    c_j = np.zeros(control_number_of_nodes)
    for i in range(0, control_number_of_nodes):
        ControlNodeList.append(ControlNode(control_ids[i], x_i[i], c_j[i]))

    ControlMesh = Mesh("control")
    ControlMesh.AddNodes(ControlNodeList)

    ## Design Geometry
    design_number_of_nodes = 2*(x_limit)+1
    DesignNodeList = []
    design_ids = np.arange(design_number_of_nodes)
    for i in range(0, design_number_of_nodes):
        DesignNodeList.append(DesignNode(design_ids[i], x_i[i], 0))

    DesignMesh = Mesh("design")
    DesignMesh.AddNodes(DesignNodeList)

    ## Optimization Set-Up
    Response = TargetGeometryResponse("target", DesignMesh, TargetMesh)

    ## Vertex Morphing Parameterization
    vm_settings = {
        "filter_radius": filter_radius,
        "integration": "RiemannSum",
        "scaling": "shape"
    }
    VM_param = VertexMorphing(DesignMesh, ControlMesh, vm_settings)

    ## Rigid Body Parameterization
    rigid_body_settings = {
        "translation": True,
        "rotation": True,
        "scaling": "shape"
    }
    RB_param = RigidBodyParameterization(DesignMesh, rigid_body_settings)

    ## Vertex Morphing + Rigid Body
    settings = {
        "scaling": "shape"
    }
    Parameterization = VertexMorphingRigidBodyParameterization(VM_param, RB_param, settings)

    step_size = 0.1
    # StepSizeSettings = ConstStepInUnscaledControl(0.5, Parameterization)
    StepSizeSettings = ConstStepInControl(step_size)

    max_steps = 20
    ConvergenceSettings = MaxSteps(max_steps)

    OptimizationAlgorithm = SteepestDescentAlgorithm("Optimierung", Parameterization, ConvergenceSettings, StepSizeSettings, NormalizeObjGrad=False)
    OptimizationAlgorithm.AddObjective(Response)

    ## Start Optimization
    OptimizationAlgorithm.StartOptimization()
    # print(OptimizationAlgorithm.Mapper.MappingMatrix)
    p = np.zeros(Parameterization.ControlSize)
    translation = [0]
    rotation = [0]
    for i in range(len(OptimizationAlgorithm.PreviousControlFields)):
        print(20*"-")
        print("optimization step {}".format(i+1))
        print("gradient {}".format(OptimizationAlgorithm.PreviousControlFields[i]["dg/dp"]))
        print("control update {}".format(OptimizationAlgorithm.PreviousControlFields[i]["delta_p"]))
        control_size = len(OptimizationAlgorithm.PreviousControlFields[i]["delta_p"])
        p = OptimizationAlgorithm.ControlParameter[i*control_size:i*control_size+control_size]
        translation.append(p[-2])
        rotation.append(p[-1])
        print("size of p: {}".format(len(p)))
        print("control values {}".format(p))
        print("objective value {}".format(OptimizationAlgorithm.PreviousObjectiveValue[i]))

    print(40*"-")
    print("final objective value {}".format(OptimizationAlgorithm.PreviousObjectiveValue[-1]))

    f = OptimizationAlgorithm.PreviousObjectiveValue

    # translation = [0]
    # rotation = [0]
    # for i in range(max_steps):
    #     translation.append(OptimizationAlgorithm.ControlParameter[2*i])
    #     rotation.append(OptimizationAlgorithm.ControlParameter[2*i+1])

    # ax.plot(translation, rotation, f, label=scaling_type)
    # # ax.plot(translation, rotation, f, label=scaling_type, marker='o')
    # ax.legend(title="scaling types")

    # Plot

    ###


    figure, axis = plt.subplots(2, 2, figsize=[5.0,8.0])
    axis[0,0].plot(x_j, p_j, '-*', color='lightgrey', label='target shape')
    axis[0,0].plot(x_j, np.zeros(len(x_j)), '-', color='lightgrey', label='initial shape')

    FinalShape = OptimizationAlgorithm.Mapper.Design
    axis[0,0].plot(FinalShape.GetNodeCoordinatesX(), FinalShape.GetShapeZ(), '-', label='design shape after {} iterations - VM+RB'.format(ConvergenceSettings.MaxSteps))
    axis[0,0].axis('equal')
    axis[0,0].legend()

    axis[0,1].plot(translation)
    axis[0,1].set(xlabel="Step", ylabel="Translation")

    axis[1,1].plot(rotation)
    axis[1,1].set(xlabel="Step", ylabel="Rotation")

    axis[1,0].plot(f, label='VM+RB')
    axis[1,0].set(xlabel="Step", ylabel="Objective")
    # figure.show()

### pure VM Optimization

## Control Geometry
x_limit = filter_radius + 4
control_number_of_nodes = 2*(x_limit)+1
x_i = np.linspace(-x_limit, x_limit, control_number_of_nodes)
ControlNodeList2 = []
control_ids = np.arange(control_number_of_nodes)
c_j = np.zeros(control_number_of_nodes)
for i in range(0, control_number_of_nodes):
    ControlNodeList2.append(ControlNode(control_ids[i], x_i[i], c_j[i]))

ControlMesh2 = Mesh("control_2")
ControlMesh2.AddNodes(ControlNodeList)

## Design Geometry
design_number_of_nodes = 2*(x_limit)+1
DesignNodeList2 = []
design_ids = np.arange(design_number_of_nodes)
for i in range(0, design_number_of_nodes):
    DesignNodeList2.append(DesignNode(design_ids[i], x_i[i], 0))

DesignMesh2 = Mesh("design_2")
DesignMesh2.AddNodes(DesignNodeList2)

## Optimization Set-Up
Response2 = TargetGeometryResponse("target", DesignMesh2, TargetMesh)

## Vertex Morphing Parameterization
vm_settings = {
    "filter_radius": filter_radius,
    "integration": "RiemannSum",
    "scaling": "none"
}
VM_pure_param = VertexMorphing(DesignMesh2, ControlMesh2, vm_settings)

# StepSizeSettings2 = ConstStepInUnscaledControl(step_size, VM_pure_param)
StepSizeSettings2 = ConstStepInControl(step_size)

ConvergenceSettings2 = MaxSteps(max_steps)

Optimization_VM = SteepestDescentAlgorithm("Optimierung", VM_pure_param, ConvergenceSettings2, StepSizeSettings2, NormalizeObjGrad=False)
Optimization_VM.AddObjective(Response2)

## Start Optimization
Optimization_VM.StartOptimization()

f2 = Optimization_VM.PreviousObjectiveValue

FinalShape2 = Optimization_VM.Mapper.Design
axis[0,0].plot(FinalShape2.GetNodeCoordinatesX(), FinalShape2.GetShapeZ(), '-', label='design shape after {} iterations - VM'.format(ConvergenceSettings.MaxSteps))
axis[0,0].axis('equal')
axis[0,0].legend()

axis[1,0].plot(f2, label='VM')
axis[1,0].legend()

plt.show()
