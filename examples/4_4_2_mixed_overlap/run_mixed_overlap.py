#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Numerical Example: Mixed Parameterization - Overlap
#####################################################################

import numpy as np
from mixedvmtool import *

def target_geometry(x_j, blending_x_min_max):
    if x_j >= -blending_x_min_max and x_j <= blending_x_min_max:
        p_j = x_j / 2 + 4 + (np.cos((x_j+blending_x_min_max)*np.pi/blending_x_min_max) - 1)
    else:
        p_j = x_j / 2 + 4

    return p_j

def CreateDesignMesh(x_limit):
    nodes_per_x = 2
    number_of_nodes = 2*nodes_per_x*(x_limit)+1
    x_i = np.linspace(-x_limit, x_limit, number_of_nodes)

    DesignNodeList = []
    design_ids = np.arange(number_of_nodes)
    for i in range(0, number_of_nodes):
        DesignNodeList.append(DesignNode(design_ids[i], x_i[i], 0))

    DesignMesh = Mesh("design")
    DesignMesh.AddNodes(DesignNodeList)

    return DesignMesh

def CreateTargetMesh(x_limit, blending_x_min_max):
    nodes_per_x = 2
    number_of_nodes = 2*nodes_per_x*(x_limit)+1
    x_i = np.linspace(-x_limit, x_limit, number_of_nodes)
    p_i = np.zeros(number_of_nodes)

    TargetNodeList = []
    target_ids = np.arange(number_of_nodes)
    for i in range(0, number_of_nodes):
        p_i[i] = target_geometry(x_i[i], blending_x_min_max)
        TargetNodeList.append(ControlNode(target_ids[i], x_i[i], p_i[i]))

    TargetMesh = Mesh("target")
    TargetMesh.AddNodes(TargetNodeList)

    return TargetMesh

def main():
    blending_x_min_max = 4
    filter_radius = 2

    ## Target Geometry
    x_limit = 12
    nodes_per_x = 2
    number_of_nodes = 2*nodes_per_x*(x_limit)+1

    x_i = np.linspace(-x_limit, x_limit, number_of_nodes)

    TargetMesh = CreateTargetMesh(x_limit, blending_x_min_max)

    # Paper scaling types
    scaling_types =["none",  "shape_diag", "shape", "shape_w_off"]
    scaling_types.reverse()

    for scaling_type in scaling_types:
        ## Control Geometry
        ControlNodeList = []
        control_ids = np.arange(number_of_nodes)
        for i in range(0, number_of_nodes):
            ControlNodeList.append(ControlNode(control_ids[i], x_i[i], 0))

        ControlMesh = Mesh("control")
        ControlMesh.AddNodes(ControlNodeList)

        ## Design Geometry
        DesignMesh = CreateDesignMesh(x_limit)

        ## Compute VM Blending
        blending_node_ids = []
        for node in DesignMesh.Nodes:
            if node.x >= -blending_x_min_max and node.x <= blending_x_min_max:
                blending_node_ids.append(node)

        vm_blending_function = DesignMesh.ComputeBlendingFunction(blending_node_ids, filter_radius)

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
        Parameterization = VertexMorphingRigidBodyParameterization(VM_param, RB_param, settings, VertexMorphingBlending=vm_blending_function)

        max_step_size = 1.0
        line_search_tolerance = 1e-4
        StepSizeSettings = GoldenSectionLineSearch(max_step_size, line_search_tolerance, Parameterization)

        max_steps = 10000
        objective_value = 1e-2
        ConvergenceSettings = ObjectiveValue(objective_value, max_steps)

        history_folder = f"results/history_scaling_{scaling_type}"
        OptimizationAlgorithm = SteepestDescentAlgorithm("Optimization with scaling type '{}'".format(scaling_type), Parameterization, ConvergenceSettings, StepSizeSettings, NormalizeObjGrad=False, HistoryFolder=history_folder)
        OptimizationAlgorithm.AddObjective(Response)

        ## Start Optimization
        OptimizationAlgorithm.StartOptimization()

if __name__ == "__main__":
    main()
