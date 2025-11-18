#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Numerical Example: Rigid Body - Variable Scaling Approaches
#####################################################################

import numpy as np
from mixedvmtool import *

def target_geometry(x_j):
    p_j = x_j / 2 + 4

    return p_j

def CreateTargetMesh(x_limit):
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

    return TargetMesh

def CreateDesignMesh(x_limit):
    ## Design Geometry
    design_number_of_nodes = 2*(x_limit)+1
    x_i = np.linspace(-x_limit, x_limit, design_number_of_nodes)
    DesignNodeList = []
    design_ids = np.arange(design_number_of_nodes)
    for i in range(0, design_number_of_nodes):
        DesignNodeList.append(DesignNode(design_ids[i], x_i[i], 0))

    DesignMesh = Mesh("design")
    DesignMesh.AddNodes(DesignNodeList)

    return DesignMesh

def main():

    x_limit = 8
    TargetMesh = CreateTargetMesh(x_limit)

    scaling_types =["none", "shape", "shape_diag_mass"]

    for scaling_type in scaling_types:

        DesignMesh = CreateDesignMesh(x_limit)

        ## Optimization Set-Up
        Response = TargetGeometryResponse("target", DesignMesh, TargetMesh)

        rigid_body_settings = {
            "translation": True,
            "rotation": True,
            "scaling": scaling_type
        }
        Mapper = RigidBodyParameterization(DesignMesh, rigid_body_settings)

        # StepSizeSettings = ConstStepInUnscaledControl(0.5, Mapper)
        StepSizeSettings = GoldenSectionLineSearch(100.0, 1e-8, Mapper)
        # StepSizeSettings = ConstStepInControl(1.0)

        max_steps = 50
        objective_value = 1e-6
        # ConvergenceSettings = MaxSteps(max_steps)
        ConvergenceSettings = ObjectiveValue(objective_value, max_steps)

        history_folder = f"results/history_scaling_{scaling_type}"
        OptimizationAlgorithm = SteepestDescentAlgorithm("Optimization with scaling type '{}'".format(scaling_type), Mapper, ConvergenceSettings, StepSizeSettings, NormalizeObjGrad=False, HistoryFolder=history_folder)
        OptimizationAlgorithm.AddObjective(Response)

        ## Start Optimization
        OptimizationAlgorithm.StartOptimization()

        print(40*"-")
        print("final objective value {}".format(OptimizationAlgorithm.PreviousObjectiveValue[-1]))
        print(40*"-")

if __name__ == "__main__":
    main()