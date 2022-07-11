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

figure, axis = plt.subplots(2, 2, figsize=[16.0,6.0])
axis[0,0].plot(x_i, p_i, '-*', color='lightgrey', label='target shape')
axis[0,0].plot(x_i, np.zeros(len(x_i)), '-', color='lightgrey', label='initial shape')
# scaling_types = ["none"]
scaling_types =["none", "shape"]

# fig, ax = create_objective_plot()

for scaling_type in scaling_types:

    ## Control Geometry
    # x_limit = filter_radius + 8
    # control_number_of_nodes = 8*(x_limit)+1
    # x_i = np.linspace(-x_limit, x_limit, number_of_nodes)
    ControlNodeList = []
    control_ids = np.arange(number_of_nodes)
    # c_i = np.zeros(number_of_nodes)
    for i in range(0, number_of_nodes):
        ControlNodeList.append(ControlNode(control_ids[i], x_i[i], 0))

    ControlMesh = Mesh("control")
    ControlMesh.AddNodes(ControlNodeList)

    ## Design Geometry
    # design_number_of_nodes = 8*(x_limit)+1
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
        "scaling": scaling_type
    }
    Parameterization = VertexMorphingRigidBodyParameterization(VM_param, RB_param, settings, VertexMorphingBlending=vm_blending_function)

    step_size = 0.1
    max_step_size = 5
    line_search_tolerance = 1e-3
    # StepSizeSettings = ConstStepInUnscaledControl(0.5, Parameterization)
    # StepSizeSettings = ConstStepInControl(step_size)
    if scaling_type == "none":
        StepSizeSettings = GoldenSectionLineSearch(0.21, line_search_tolerance, Parameterization)
    else:
        StepSizeSettings = GoldenSectionLineSearch(max_step_size, line_search_tolerance, Parameterization)

    max_steps = 2000
    ConvergenceSettings = MaxSteps(max_steps)

    OptimizationAlgorithm = SteepestDescentAlgorithm("Optimization with scaling type '{}'".format(scaling_type), Parameterization, ConvergenceSettings, StepSizeSettings, NormalizeObjGrad=False)
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
        # print("gradient {}".format(OptimizationAlgorithm.PreviousControlFields[i]["dg/dp"]))
        # print("control update {}".format(OptimizationAlgorithm.PreviousControlFields[i]["delta_p"]))
        control_size = len(OptimizationAlgorithm.PreviousControlFields[i]["delta_p"])
        p = OptimizationAlgorithm.ControlParameter[i*control_size:i*control_size+control_size]
        translation.append(p[-2])
        rotation.append(p[-1])
        print("control values {}".format(p))
        print("objective value {}".format(OptimizationAlgorithm.PreviousObjectiveValue[i]))

    print(40*"-")
    print("final objective value {}".format(OptimizationAlgorithm.PreviousObjectiveValue[-1]))

    f = OptimizationAlgorithm.PreviousObjectiveValue

    FinalShape = OptimizationAlgorithm.Mapper.Design
    axis[0,0].plot(FinalShape.GetNodeCoordinatesX(), FinalShape.GetShapeZ(), '-', label='shape after {} iterations - VM+RB with scaling type: {}'.format(ConvergenceSettings.MaxSteps, scaling_type))
    axis[0,0].axis('equal')
    axis[0,0].legend()

    axis[0,1].plot(translation, label="Scaling type: {}".format(scaling_type))
    axis[0,1].set(xlabel="Step", ylabel="Translation ")

    axis[1,1].plot(rotation, label="Scaling type: {}".format(scaling_type))
    axis[1,1].set(xlabel="Step", ylabel="Rotation")

    axis[1,0].plot(f, label='VM+RB with scaling_type: {}'.format(scaling_type))
    axis[1,0].set(xlabel="Step", ylabel="Objective")

### pure VM Optimization

## Control Geometry
# x_limit = filter_radius + 8
# control_number_of_nodes = 2*(x_limit)+1
# x_i = np.linspace(-x_limit, x_limit, number_of_nodes)
ControlNodeList2 = []
control_ids = np.arange(number_of_nodes)
# c_j = np.zeros(number_of_nodes)
for i in range(0, number_of_nodes):
    ControlNodeList2.append(ControlNode(control_ids[i], x_i[i],0))

ControlMesh2 = Mesh("control_2")
ControlMesh2.AddNodes(ControlNodeList)

## Design Geometry
# design_number_of_nodes = 2*(x_limit)+1
DesignNodeList2 = []
design_ids = np.arange(number_of_nodes)
for i in range(0, number_of_nodes):
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
# StepSizeSettings2 = ConstStepInControl(step_size)
StepSizeSettings2 = GoldenSectionLineSearch(max_step_size, line_search_tolerance, VM_pure_param)

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
axis[0,1].legend()
axis[1,1].legend()

axis[1,0].set_yscale('log')
plt.show()
