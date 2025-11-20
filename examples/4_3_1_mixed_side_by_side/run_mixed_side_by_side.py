#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Numerical Example: Mixed Parameterization - Side by Side
#####################################################################

import numpy as np
from mixedvmtool import *

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

def CreateTargetMesh(x_limit, blending_filter):
    blending_vm_x_min_max = [0, int(4+(x_limit-4)/2-blending_filter/2)]
    blending_rb_x_min_max = [int(4+(x_limit-4)/2+blending_filter/2), x_limit]

    ## Target Geometry
    nodes_per_x = 1
    number_of_nodes = nodes_per_x*(x_limit)+1
    x_i = np.linspace(0, x_limit, number_of_nodes)

    TargetNodeList = []
    target_ids = np.arange(number_of_nodes)
    for i in range(0, number_of_nodes):
        TargetNodeList.append(ControlNode(target_ids[i], x_i[i], 0))

    TargetMesh = Mesh("target")
    TargetMesh.AddNodes(TargetNodeList)

    for node in TargetMesh.Nodes:
        node.z = target_geometry(node.x, blending_filter,
                                 blending_rb_x_min_max, blending_vm_x_min_max)

    return TargetMesh

def CreateDesignMesh(x_limit):
    nodes_per_x = 1
    number_of_nodes = nodes_per_x*(x_limit)+1
    x_i = np.linspace(0, x_limit, number_of_nodes)

    DesignNodeList = []
    design_ids = np.arange(number_of_nodes)
    for i in range(0, number_of_nodes):
        DesignNodeList.append(DesignNode(design_ids[i], x_i[i], 0))

    DesignMesh = Mesh("design")
    DesignMesh.AddNodes(DesignNodeList)

    return DesignMesh

def main():
    filter_radius = 4
    blending_filter = 4
    x_limit = 4+24
    nodes_per_x = 1
    number_of_nodes = nodes_per_x*(x_limit)+1
    x_i = np.linspace(0, x_limit, number_of_nodes)

    blending_vm_x_min_max = [0, int(4+(x_limit-4)/2-blending_filter/2)]
    blending_rb_x_min_max = [int(4+(x_limit-4)/2+blending_filter/2), x_limit]
    rb_rotation_center = (blending_rb_x_min_max[1] - blending_rb_x_min_max[0]) / 2 + blending_rb_x_min_max[0]

    ## Target Geometry
    TargetMesh = CreateTargetMesh(x_limit, blending_filter)

    # ## Compute VM Blending
    # blending_vm_nodes = []
    # blending_rb_nodes = []
    # for node in TargetMesh.Nodes:
    #     if node.x >= blending_vm_x_min_max[0] and node.x <= blending_vm_x_min_max[1]:
    #         blending_vm_nodes.append(node)
    #     if node.x >= blending_rb_x_min_max[0] and node.x <= blending_rb_x_min_max[1]:
    #         blending_rb_nodes.append(node)

    # blending_vm = TargetMesh.ComputeBlendingFunction(blending_vm_nodes, blending_filter)
    # blending_rb = TargetMesh.ComputeBlendingFunction(blending_rb_nodes, blending_filter)

    # scaling types
    scaling_types =["none",  "shape_diag", "shape", "shape_w_off"]
    scaling_types.reverse()

    for scaling_type in scaling_types:

        ## Control Geometry
        ControlNodeList = []
        control_ids = np.arange(number_of_nodes)
        for i in range(0, number_of_nodes):
            ControlNodeList.append(DesignNode(control_ids[i], x_i[i], 0))

        ControlMesh = Mesh("control")
        ControlMesh.AddNodes(ControlNodeList)

        ## Design Geometry
        DesignMesh = CreateDesignMesh(x_limit)

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

        ## Mixed Parameterization: Vertex Morphing + Rigid Body
        settings = {
                "scaling": scaling_type
            }
        Parameterization = VertexMorphingRigidBodyParameterization(VM_param, RB_param, settings, VertexMorphingBlending=blending_vm, RigidBodyBlending=blending_rb)

        max_step_size = 1.0
        line_search_tolerance = 1e-3
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