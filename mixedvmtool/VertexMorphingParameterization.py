#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Mapping Matrix
#####################################################################

# external imports
import numpy as np
import time
# internal imports
from .ShapeFunctions import *
from .GaussianQuadrature import *
from .Node import *
from .Mesh import Mesh

class RiemannSum():

    def __init__(self, DesignMesh, ControlMesh, settings):

        self.Design = DesignMesh
        self.Control = ControlMesh
        self.FilterRadius = settings["filter_radius"]

        self.MappingMatrix = np.zeros([len(self.Design.Nodes),
                                       len(self.Control.Nodes)])

        self.ControlSize = len(self.Control.Nodes)

        self.domain_edges = [self.Design.Nodes[0].x, self.Design.Nodes[-1].x]

    def CalculateMappingMatrix(self):

        print("Vertex Morphing - CalculateMappingMatrix: Starting")
        start = time.time()
        nodal_areas = self.Control.ComputeNodalAreas()
        max_sum_of_weights = 0
        node_x = self.Control.GetNodeCoordinatesX()

        for design_node_index, design_node in enumerate(self.Design.Nodes):

            control_neighbour_node_indices = self.Control.GetNodeInFilterRadius(design_node.x, node_x, self.FilterRadius, self.FilterRadius)
            weights = np.zeros(len(control_neighbour_node_indices))
            sum_of_weights = 0

            for i in range(len(control_neighbour_node_indices)):

                # control_neighbour_node = control_neighbour_nodes[i]
                neighbour_nodes_index = control_neighbour_node_indices[i]

                nodal_area_i = nodal_areas[neighbour_nodes_index]

                weight = nodal_area_i * LinearFilter(node_x[neighbour_nodes_index], design_node.x, self.FilterRadius)
                weights[i] = weight
                sum_of_weights += weight

                self.MappingMatrix[design_node_index, neighbour_nodes_index] += weights[i]

            if abs(sum_of_weights) > 1e-16:
                self.MappingMatrix[design_node_index, :] /= sum_of_weights

        end = time.time()
        print(f"Vertex Morphing - CalculateMappingMatrix: Finished in {end - start}s")

        return self.MappingMatrix

class VertexMorphing():

    def __init__(self, DesignMesh, ControlMesh, settings):

        self.Design = DesignMesh
        self.Control = ControlMesh
        self.FilterRadius = settings["filter_radius"]
        self.scaling_type = settings["scaling"]

        self.MappingMatrix = np.zeros([len(self.Design.Nodes),
                                       len(self.Control.Nodes)])

        self.ControlSize = len(self.Control.Nodes)

        if settings["integration"] == "RiemannSum":
            self.IntegrationMethod = RiemannSum(self.Design, self.Control, settings)
        elif settings["integration"] == "GaussianQuadrature":
            self.IntegrationMethod = GaussianIntegration(self.Design, self.Control, settings)
        else:
            ValueError("'scaling_type' unknown!")

        self.scaling_calculation_time = 0

    def Calculate(self, blending=None):

        self.MappingMatrix = self.IntegrationMethod.CalculateMappingMatrix()
        if blending is not None:
            self.MappingMatrix *= blending[:, np.newaxis]

        if self.scaling_type == "none" or self.scaling_type == "pseudo_inv":
            self.scaling_matrix = np.eye(self.ControlSize)
        elif self.scaling_type == "shape":
            nodal_areas = self.Design.ComputeNodalAreas()
            mass_matrix = self.CalculateDiagonalMassMatrix(nodal_areas, self.MappingMatrix)
            start = time.time()
            self.scaling_matrix = self.CalculateDiagonalVariableScalingMatrix(mass_matrix)
            end = time.time()
        elif self.scaling_type == "shape_non_diag":
            nodal_areas = self.Design.ComputeNodalAreas()
            mass_matrix = self.CalculateMassMatrix(nodal_areas, self.MappingMatrix)
            start = time.time()
            self.scaling_matrix = self.CalculateVariableScalingMatrix(mass_matrix)
            end = time.time()
        else:
            ValueError("'scaling_type' unknown!")

        if self.scaling_type != "none":
            self.scaling_calculation_time = end - start

    def MapGradient(self, gradient):

        if self.scaling_type == "none":
            mapped_gradient = self.MappingMatrix.transpose() @ gradient
        elif self.scaling_type == "column" or self.scaling_type == "shape" or self.scaling_type == "shape_non_diag":
            mapped_gradient = self.scaling_matrix.transpose() @ self.MappingMatrix.transpose() @ gradient
        elif self.scaling_type == "pseudo_inv":
            mapped_gradient = np.linalg.pinv(self.MappingMatrix) @ gradient

        return mapped_gradient

    def MapUpdate(self, control_update):

        if self.scaling_type == "none" or self.scaling_type == "pseudo_inv":
            design_update = self.MappingMatrix @ control_update
        elif self.scaling_type == "column" or self.scaling_type == "shape" or self.scaling_type == "shape_non_diag":
            design_update = self.MappingMatrix @ self.scaling_matrix @ control_update

        return design_update

    def GetUnscaledControlParameter(self, control_update):

        if self.scaling_type == "none" or self.scaling_type == "pseudo_inv":
            unscaled_control_update = control_update
        elif self.scaling_type == "column" or self.scaling_type == "shape" or self.scaling_type == "shape_non_diag":
            unscaled_control_update = self.scaling_matrix @ control_update

        return unscaled_control_update

    @staticmethod
    def CalculateDiagonalMassMatrix(nodal_areas, matrix):
        M = np.zeros((matrix.shape[1], matrix.shape[1]))

        M = np.diag((matrix * nodal_areas[:, None]).sum(axis=0))

        diagonal = M.diagonal()
        tolerance = 1e-8
        mask = np.isclose(diagonal, 0.0, atol=tolerance)
        if np.any(mask):
            M[mask, mask] = 1.0

        # set all entries < 1e-8 to 0
        bad_indices = abs(M) < 1e-8
        M[bad_indices] = 0

        return M

    @staticmethod
    def CalculateMassMatrix(nodal_areas, matrix):
        M = np.zeros((matrix.shape[1], matrix.shape[1]))

        weighted_matrix = matrix * nodal_areas[:, None]
        M = matrix.T @ weighted_matrix

        diagonal = M.diagonal()
        tolerance = 1e-8
        mask = np.isclose(diagonal, 0.0, atol=tolerance)
        if np.any(mask):
            M[mask, mask] = 1.0

        # set all entries < 1e-8 to 0
        bad_indices = abs(M) < 1e-8
        M[bad_indices] = 0

        return M

    @staticmethod
    def CalculateVariableScalingMatrix(mass_matrix):
        eigvals, eigvecs = np.linalg.eigh(np.linalg.inv(mass_matrix))

        for i in range(eigvecs.shape[0]):
            if eigvecs[i, i] < 0:
                eigvecs[:, i] *= -1

        # P @ D @ P^T = mass_matrix^-1
        P = eigvecs
        D = np.diag(eigvals)

        return P @ np.sqrt(D)  # this is applied to update and the transpose to gradients

    @staticmethod
    def CalculateDiagonalVariableScalingMatrix(mass_matrix):

        S = np.zeros(mass_matrix.shape)

        np.fill_diagonal(S, np.sqrt(np.reciprocal(np.diag(mass_matrix))))

        return S  # this is applied to update and the transpose to gradients
