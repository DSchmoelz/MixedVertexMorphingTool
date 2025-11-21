#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Vertex Morphing + Rigid Body Parameterization
#####################################################################

# external imports
import numpy as np
import time
# internal imports
from .Node import *
from .Mesh import Mesh

class VertexMorphingRigidBodyParameterization():

    def __init__(self, VertexMorphing, RigidBody, settings, VertexMorphingBlending=None, RigidBodyBlending=None):

        """
        settings: {
            "translation": True,
            "rotation": True,
            "scaling": "none"
        }
        """
        self.RigidBody = RigidBody
        self.VertexMorphing = VertexMorphing
        self.Design = RigidBody.Design
        if RigidBodyBlending is None:
            self.RigidBodyBlending = np.ones(len(self.Design.Nodes))
        else:
            self.RigidBodyBlending = RigidBodyBlending
        if VertexMorphingBlending is None:
            self.VertexMorphingBlending = np.ones(len(self.Design.Nodes))
        else:
            self.VertexMorphingBlending = VertexMorphingBlending

        # control is numpy array of parameter [Translation, Rotation]
        self.ControlSize = VertexMorphing.ControlSize + RigidBody.ControlSize

        self.Control = np.zeros(self.ControlSize)

        self.scaling_type = settings["scaling"]
        self.scaling_calculation_time = 0

    def Calculate(self):

        self.MappingMatrix = np.zeros([len(self.Design.Nodes),
                                       self.ControlSize])

        self.VertexMorphing.Calculate(self.VertexMorphingBlending)

        computed_parameters = 0
        self.MappingMatrix[:,
                           computed_parameters:computed_parameters+self.VertexMorphing.ControlSize] = self.VertexMorphing.MappingMatrix

        computed_parameters += self.VertexMorphing.ControlSize
        self.RigidBody.Calculate(self.RigidBodyBlending)

        self.MappingMatrix[:,
                           computed_parameters:computed_parameters+self.RigidBody.ControlSize] = self.RigidBody.MappingMatrix

        self.scaling_matrix = np.eye(self.ControlSize)

        if self.scaling_type == "shape_w_off":
            nodal_areas = self.Design.ComputeNodalAreas()
            mass_matrix = np.zeros((self.ControlSize, self.ControlSize))
            computed_parameters = 0
            mass_matrix[computed_parameters:computed_parameters+self.VertexMorphing.ControlSize,
                                computed_parameters:computed_parameters+self.VertexMorphing.ControlSize] = self.VertexMorphing.CalculateDiagonalMassMatrix(nodal_areas, self.VertexMorphing.MappingMatrix)
            computed_parameters += self.VertexMorphing.ControlSize

            if self.scaling_type == "shape_w_off":
                mass_matrix_off_diag = self.CalculateMassMatrixOffDiagonal(nodal_areas, self.VertexMorphing.MappingMatrix, self.RigidBody.MappingMatrix)
                mass_matrix[0:self.VertexMorphing.ControlSize,
                            computed_parameters:computed_parameters+self.RigidBody.ControlSize] = mass_matrix_off_diag

                mass_matrix[computed_parameters:computed_parameters+self.RigidBody.ControlSize,
                            0:self.VertexMorphing.ControlSize] = mass_matrix_off_diag.transpose()

            mass_matrix[computed_parameters:computed_parameters+self.RigidBody.ControlSize,
                        computed_parameters:computed_parameters+self.RigidBody.ControlSize] = self.CalculateMassMatrix(nodal_areas, self.RigidBody.MappingMatrix)

            start = time.time()
            self.scaling_matrix = self.CalculateVariableScalingMatrix(mass_matrix)
            end = time.time()
            self.scaling_calculation_time = end - start

        elif self.scaling_type in ["shape", "shape_diag"]:
            self.scaling_calculation_time = (self.VertexMorphing.scaling_calculation_time +
                                             self.RigidBody.scaling_calculation_time)

            self.scaling_matrix = np.zeros((self.ControlSize, self.ControlSize))
            computed_parameters = 0
            self.scaling_matrix[computed_parameters:computed_parameters+self.VertexMorphing.ControlSize,
                                computed_parameters:computed_parameters+self.VertexMorphing.ControlSize] = self.VertexMorphing.scaling_matrix

            computed_parameters += self.VertexMorphing.ControlSize

            self.scaling_matrix[computed_parameters:computed_parameters+self.RigidBody.ControlSize,
                                computed_parameters:computed_parameters+self.RigidBody.ControlSize] = self.RigidBody.scaling_matrix

    def MapGradient(self, gradient):

        if self.scaling_type == "none":
            mapped_gradient = self.MappingMatrix.transpose() @ gradient
        elif self.scaling_type == "column" or self.scaling_type in ["shape", "shape_diag", "shape_w_off"]:
            mapped_gradient = self.scaling_matrix.transpose() @ self.MappingMatrix.transpose() @ gradient
        elif self.scaling_type == "pseudo_inv":
            mapped_gradient = np.linalg.pinv(self.MappingMatrix @ self.scaling_matrix) @ gradient

        return mapped_gradient

    def MapUpdate(self, control_update):

        if self.scaling_type == "none":
            design_update = self.MappingMatrix @ control_update
        elif self.scaling_type == "column" or self.scaling_type in ["shape", "shape_diag", "shape_w_off"]:
            design_update = self.MappingMatrix @ self.scaling_matrix @ control_update
        elif self.scaling_type == "pseudo_inv":
            design_update = self.MappingMatrix @ (self.scaling_matrix @ control_update)

        return design_update

    def GetUnscaledControlParameter(self, control_update):

        if self.scaling_type == "none" or self.scaling_type == "pseudo_inv":
            unscaled_control_update = control_update
        elif self.scaling_type == "column" or self.scaling_type in ["shape", "shape_diag", "shape_w_off"]:
            unscaled_control_update = self.scaling_matrix @ control_update

        return unscaled_control_update

    def GetCenterOfGravity(self):

        nodal_areas = self.Design.ComputeNodalAreas()
        node_coordinates = self.Design.GetNodeCoordinatesX()

        center_of_gravity = np.dot(nodal_areas, node_coordinates) / np.sum(nodal_areas)

        return center_of_gravity

    @staticmethod
    def CalculateMassMatrixOffDiagonal(nodal_areas, matrix_a, matrix_b):
        M = np.zeros((matrix_a.shape[1], matrix_b.shape[1]))

        weighted_b = matrix_b * nodal_areas[:, None]
        M = matrix_a.T @ weighted_b

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

