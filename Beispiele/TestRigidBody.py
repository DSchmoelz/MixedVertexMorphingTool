#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# TestSteepestDescentAlgorithm
#####################################################################

import numpy as np
import sys
import path_setting
sys.path.append(path_setting.path)
from vmtool import *
import matplotlib.pyplot as plt

def target_geometry(x_j):
    p_j = x_j / 2 + 4

    return p_j

## Design Geometry
filter_radius = 4
x_limit = filter_radius + 4
design_number_of_nodes = 2*(x_limit)+1
x_i = np.linspace(-x_limit, x_limit, design_number_of_nodes)

DesignNodeList = []
design_ids = np.arange(design_number_of_nodes)
for i in range(0, design_number_of_nodes):
    DesignNodeList.append(DesignNode(design_ids[i], x_i[i], 0))

DesignMesh = Mesh("design")
DesignMesh.AddNodes(DesignNodeList)


## Target Geometry
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


## Control Geometry
ControlNodeList = []
control_number_of_nodes = design_number_of_nodes
x_p = np.linspace(-4, 4, design_number_of_nodes)
control_ids = np.arange(control_number_of_nodes)
for i in range(0, control_number_of_nodes):
    ControlNodeList.append(ControlNode(control_ids[i], x_p[i], 0))

ControlMesh = Mesh("control")
ControlMesh.AddNodes(ControlNodeList)

## Optimization Set-Up
Response = TargetGeometryResponse("target", DesignMesh, TargetMesh)

rigid_body_settings = {
    "translation": True,
    "rotation": True,
    "scaling": "shape"
}
Mapper = RigidBodyParameterization(DesignMesh, rigid_body_settings)
StepSizeSettings = ConstStepInControl(1.0)
ConvergenceSettings = MaxSteps(10)

OptimizationAlgorithm = SteepestDescentAlgorithm("Optimierung", Mapper, ConvergenceSettings, StepSizeSettings, NormalizeObjGrad=True)
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
## Plot
fig, axis = plt.subplots(2, figsize=[5.0,8.0])
axis[0].plot(x_j, p_j, '-*', color='lightgrey', label='target shape')

FinalShape = OptimizationAlgorithm.Mapper.Design
axis[0].plot(FinalShape.GetNodeCoordinatesX(), FinalShape.GetShapeZ(), '-', label='design shape after {} iterations'.format(ConvergenceSettings.MaxSteps))
axis[0].axis('equal')
axis[0].legend()

axis[1].plot(f)
axis[1].set(xlabel="Step", ylabel="Objective")
plt.show()
