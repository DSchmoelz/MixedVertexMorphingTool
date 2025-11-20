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


class GaussianIntegration():

    def __init__(self, DesignMesh, ControlMesh, settings):

        self.Design = DesignMesh
        self.Control = ControlMesh
        self.FilterRadius = settings["filter_radius"]

        self.MappingMatrix = np.zeros([len(self.Design.Nodes),
                                       len(self.Control.Nodes)])
        self.nGP = 2
        self.ControlSpace = self.Control.Space()
        self.DesignSpace = self.Design.Space()
        self.ControlSize = len(self.Control.Nodes)

    def CalculateMappingMatrix(self):
        for design_node in self.Design.Nodes:

            integration_intervals = self.GetIntegrationIntervals(design_node)

            for interval in integration_intervals:
                gauss_points, gauss_weights = CalculateGaussPointsAndWeights(self.nGP, interval[0], interval[1])
                interval_length = interval[1] - interval[0]

                for control_node in self.Control.Nodes:
                    for gauss_point, gauss_weight in zip(gauss_points, gauss_weights):

                        self.MappingMatrix[self.Design.GetNodeIndex(design_node.id), self.Control.GetNodeIndex(control_node.id)] += interval_length/2 * LinearHatFunction(gauss_point, design_node.x, self.FilterRadius) * self.Control.GetShapeFunctionValueOfNode(gauss_point, control_node) * gauss_weight

        return self.MappingMatrix

    def GetIntegrationIntervals(self, node):

        # find nodes in filter
        node_x = self.Control.GetNodeCoordinatesX()
        nodes_in_filter = self.Control.GetNodeInFilterRadius(node.x, node_x, self.FilterRadius, self.FilterRadius)

        position_of_nodes_in_filter = []
        for node_in_filter in nodes_in_filter:
            position_of_nodes_in_filter.append(node_in_filter.x)

        interval_points = position_of_nodes_in_filter

        # add node position for filter itself
        interval_points.append(node.x)
        # add filter interval left
        filter_start = node.x - self.FilterRadius
        interval_points.append(filter_start)
        # add filter interval right
        filter_end = node.x + self.FilterRadius
        interval_points.append(filter_end)

        interval_points.sort()

        # remove duplicates from list
        interval_points = list(dict.fromkeys(interval_points))
        intervals = np.zeros((len(interval_points)-1, 2))

        for i in range(len(interval_points)-1):
            intervals[i,:] = interval_points[i:i+2]

        return intervals

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

        nodal_areas = self.Control.ComputeNodalAreas()
        max_sum_of_weights = 0
        node_x = self.Control.GetNodeCoordinatesX()

        for design_node in self.Design.Nodes:

            control_neighbour_nodes = self.Control.GetNodeInFilterRadius(design_node.x, node_x, self.FilterRadius, self.FilterRadius)
            weights = np.zeros(len(control_neighbour_nodes))
            sum_of_weights = 0

            for i in range(len(control_neighbour_nodes)):

                control_neighbour_node = control_neighbour_nodes[i]

                neighbour_nodes_index = self.Control.GetNodeIndex(control_neighbour_node.id)
                nodal_area_i = nodal_areas[neighbour_nodes_index]

                weight = nodal_area_i * LinearFilter(control_neighbour_node.x, design_node.x, self.FilterRadius)
                weights[i] = weight
                sum_of_weights += weight

                self.MappingMatrix[self.Design.GetNodeIndex(design_node.id), self.Control.GetNodeIndex(control_neighbour_node.id)] += weights[i]

            if abs(sum_of_weights) > 1e-16:
                self.MappingMatrix[self.Design.GetNodeIndex(design_node.id), :] /= sum_of_weights

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
        for i in range(matrix.shape[1]):
            M[i, i] = np.sum(np.multiply(matrix[:, i], nodal_areas))

        diagonal = M.diagonal()
        tolerance = 1e-8
        mask = np.isclose(diagonal, 0.0, atol=tolerance)
        if np.any(mask):
            M[mask, mask] = 1.0

        # if np.any(diagonal == 0.0):
        #     M[diagonal == 0.0, diagonal == 0.0] = 1.0

        # set all entries < 1e-8 to 0
        bad_indices = abs(M) < 1e-8
        M[bad_indices] = 0

        return M

    @staticmethod
    def CalculateMassMatrix(nodal_areas, matrix):
        M = np.zeros((matrix.shape[1], matrix.shape[1]))
        for i in range(matrix.shape[1]):
            for j in range(matrix.shape[1]):
                M[i, j] = matrix[:, i] @ np.multiply(matrix[:, j], nodal_areas)

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

        # for i in range(mass_matrix.shape[0]):
        #     S[i,i] = np.sqrt((1/mass_matrix[i,i]))

        return S  # this is applied to update and the transpose to gradients
