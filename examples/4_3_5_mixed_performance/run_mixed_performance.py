#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Numerical Example: Mixed Parameterization - Performance Test
#####################################################################

import numpy as np
import pandas as pd
from mixedvmtool import *

def CreateTargetMesh(x_limit, number_of_nodes):
    ## Target Geometry
    x_i = np.linspace(0, x_limit, number_of_nodes)

    TargetNodeList = []
    target_ids = np.arange(number_of_nodes)
    for i in range(0, number_of_nodes):
        TargetNodeList.append(ControlNode(target_ids[i], x_i[i], 0))

    TargetMesh = Mesh("target")
    TargetMesh.AddNodes(TargetNodeList)

    for node in TargetMesh.Nodes:
        node.z = 1

    return TargetMesh

def CreateDesignMesh(x_limit, number_of_nodes):
    x_i = np.linspace(0, x_limit, number_of_nodes)

    DesignNodeList = []
    design_ids = np.arange(number_of_nodes)
    for i in range(0, number_of_nodes):
        DesignNodeList.append(DesignNode(design_ids[i], x_i[i], 0))

    DesignMesh = Mesh("design")
    DesignMesh.AddNodes(DesignNodeList)

    return DesignMesh

def CalculateMappingMatrix(MixedParameterization):

    MixedParameterization.MappingMatrix = np.zeros([len(MixedParameterization.Design.Nodes),
                                    MixedParameterization.ControlSize])

    MixedParameterization.VertexMorphing.Calculate(MixedParameterization.VertexMorphingBlending)

    computed_parameters = 0
    MixedParameterization.MappingMatrix[:,
                        computed_parameters:computed_parameters+MixedParameterization.VertexMorphing.ControlSize] = MixedParameterization.VertexMorphing.MappingMatrix

    computed_parameters += MixedParameterization.VertexMorphing.ControlSize
    MixedParameterization.RigidBody.Calculate(MixedParameterization.RigidBodyBlending)

    MixedParameterization.MappingMatrix[:,
                        computed_parameters:computed_parameters+MixedParameterization.RigidBody.ControlSize] = MixedParameterization.RigidBody.MappingMatrix

def CalculateMassMatrixWithOffDiagonal(MixedParameterization):

    nodal_areas = MixedParameterization.Design.ComputeNodalAreas()
    mass_matrix = np.zeros((MixedParameterization.ControlSize, MixedParameterization.ControlSize))
    computed_parameters = 0
    mass_matrix[computed_parameters:computed_parameters+MixedParameterization.VertexMorphing.ControlSize,
                        computed_parameters:computed_parameters+MixedParameterization.VertexMorphing.ControlSize] = MixedParameterization.VertexMorphing.CalculateDiagonalMassMatrix(nodal_areas, MixedParameterization.VertexMorphing.MappingMatrix)
    computed_parameters += MixedParameterization.VertexMorphing.ControlSize

    mass_matrix_off_diag = MixedParameterization.CalculateMassMatrixOffDiagonal(nodal_areas, MixedParameterization.VertexMorphing.MappingMatrix, MixedParameterization.RigidBody.MappingMatrix)
    mass_matrix[0:MixedParameterization.VertexMorphing.ControlSize,
                computed_parameters:computed_parameters+MixedParameterization.RigidBody.ControlSize] = mass_matrix_off_diag

    mass_matrix[computed_parameters:computed_parameters+MixedParameterization.RigidBody.ControlSize,
                0:MixedParameterization.VertexMorphing.ControlSize] = mass_matrix_off_diag.transpose()

    mass_matrix[computed_parameters:computed_parameters+MixedParameterization.RigidBody.ControlSize,
            computed_parameters:computed_parameters+MixedParameterization.RigidBody.ControlSize] = MixedParameterization.RigidBody.CalculateMassMatrix(nodal_areas, MixedParameterization.RigidBody.MappingMatrix)

    return mass_matrix

def main():
    # blending_filters = [10, 20, 40, 80]
    blending_filters = [80]
    # element_sizes = [1, 0.5, 0.25, 0.1, 0.05, 0.025, 0.01]
    element_sizes = [1, 0.5, 0.25, 0.1, 0.05, 0.025, 0.01, 0.005, 0.0025, 0.001]
    # element_sizes = [0.05]

    for blending_filter in blending_filters:
        print(100*"-")
        print(f"BLENDING SIZE: {blending_filter}")
        result_times = {
            "shape_w_off": [],
            "shape": [],
            "mapping": [],
            "mass": [],
        }

        for iter, element_size in enumerate(element_sizes):
            print(50*"-")
            print(f"ELEMENT SIZE: {element_size}")
            filter_radius = 5
            x_limit = 100
            number_of_nodes = int(x_limit/element_size) + 1
            x_i = np.linspace(0, x_limit, number_of_nodes)

            blending_vm_x_min_max = [0, int(x_limit/2-blending_filter/2)]
            blending_rb_x_min_max = [int(x_limit/2+blending_filter/2), x_limit]

            ## Target Geometry
            TargetMesh = CreateTargetMesh(x_limit, number_of_nodes)

            ## Control Geometry
            ControlNodeList = []
            control_ids = np.arange(number_of_nodes)
            for i in range(0, number_of_nodes):
                ControlNodeList.append(DesignNode(control_ids[i], x_i[i], 0))

            ControlMesh = Mesh("control")
            ControlMesh.AddNodes(ControlNodeList)

            ## Design Geometry
            DesignMesh = CreateDesignMesh(x_limit, number_of_nodes)

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

            ## Vertex Morphing Parameterization
            vm_settings = {
                "filter_radius": filter_radius,
                "integration": "RiemannSum",
                "scaling": "none"
            }
            VM_param = VertexMorphing(DesignMesh, ControlMesh, vm_settings)

            ## Rigid Body Parameterization
            rigid_body_settings = {
                "translation": True,
                "rotation": True,
                "scaling": "none"
            }
            RB_param = RigidBodyParameterization(DesignMesh, rigid_body_settings)

            ## Mixed Parameterization: Vertex Morphing + Rigid Body
            settings = {
                    "scaling": "none"
                }
            Parameterization = VertexMorphingRigidBodyParameterization(VM_param, RB_param, settings, VertexMorphingBlending=blending_vm, RigidBodyBlending=blending_rb)

            start = time.time()
            CalculateMappingMatrix(Parameterization)
            end = time.time()
            print(f"Mapping matrix computation time: {end-start}")
            result_times["mapping"].append(end-start)

            start = time.time()
            mass_matrix = CalculateMassMatrixWithOffDiagonal(Parameterization)
            end = time.time()
            print(f"Mass matrix computation time: {end-start}")
            result_times["mass"].append(end-start)

            # print(f"mass matrix vm:\n{mass_matrix[:-2,:-2]}")
            # print(f"mass matrix rb:\n{mass_matrix[-2:,-2:]}")
            # print(f"mass matrix off:\n{mass_matrix[:-2,-2:]}")

            # compute variable scaling matrix with proposed mass matrix and measure time
            start = time.time()
            Parameterization.VertexMorphing.CalculateDiagonalVariableScalingMatrix(mass_matrix[:-2,:-2])
            Parameterization.RigidBody.CalculateVariableScalingMatrix(mass_matrix[-2:,-2:])
            end = time.time()
            calc_time = end - start
            print(f"shape: Scaling computation time: {calc_time}")
            result_times["shape"].append(calc_time)

            # compute variable scaling matrix with mass matrix w off-diagonal and measure time
            start = time.time()
            Parameterization.CalculateVariableScalingMatrix(mass_matrix)
            end = time.time()
            calc_time = end - start
            print(f"shape_w_off: Scaling computation time: {calc_time}")
            result_times["shape_w_off"].append(calc_time)

            # save in .csv
            df = pd.DataFrame(result_times, index=element_sizes[:iter+1])
            df.index.name = "element_sizes"
            df.to_csv(f"blending{blending_filter}_scaling_calc_times_v2.csv")

if __name__ == "__main__":
    main()