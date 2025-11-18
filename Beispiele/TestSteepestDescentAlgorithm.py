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
    if x_j < 0:
        p_j = 3/4 * x_j + 3
    elif x_j > 0:
        p_j = -3/4 * x_j + 3
    else:
        p_j = 3
    return p_j

## Target Geometry
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

## Design Geometry
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

## Control Geometry
ControlNodeList = []
control_number_of_nodes = design_number_of_nodes
x_p = np.linspace(-4, 4, design_number_of_nodes)
control_ids = np.arange(control_number_of_nodes)
for i in range(0, control_number_of_nodes):
    ControlNodeList.append(ControlNode(control_ids[i], x_p[i], 0))

# ControlNodeList.append(ControlNode(i+1, 0, 0))
# ControlNodeList.append(ControlNode(i+2, 0, 0))
# ControlNodeList.append(ControlNode(i+3, 0, 0))

ControlMesh = Mesh("control")
ControlMesh.AddNodes(ControlNodeList)

## Optimization Set-Up
Response = TargetGeometryResponse("target", DesignMesh, TargetMesh)

## TODO: Mit Gaussian Integration Funktioniert Optimierung noch nicht!! Warum??
settings = {
    "filter_radius": filter_radius,
    "scaling": "pseudo_inv",
    "integration": "RiemannSum"
}
Mapper = VertexMorphing(DesignMesh, ControlMesh, settings)
StepSizeSettings = ConstStepInControl(0.2)
ConvergenceSettings = MaxSteps(5)

OptimizationAlgorithm = SteepestDescentAlgorithm("Optimierung", Mapper, ConvergenceSettings, StepSizeSettings)
OptimizationAlgorithm.AddObjective(Response)

## Start Optimization
OptimizationAlgorithm.StartOptimization()

## Plot
plt.figure()
plt.plot(x_j, p_j, '-*', color='lightgrey', label='target shape')

FinalShape = OptimizationAlgorithm.Mapper.Design
plt.plot(FinalShape.GetNodeCoordinatesX(), FinalShape.GetShapeZ(), '-', label='design shape with r = {} after {} iterations'.format(filter_radius, ConvergenceSettings.MaxSteps))

plt.legend()
plt.show()
