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
# internal imports
from .Node import *
from .Mesh import Mesh

class VertexMorphingRigidBodyParameterization():

    def __init__(self, VertexMorphing, RigidBody, settings):

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
        # control is numpy array of parameter [Translation, Rotation]
        self.ControlSize = VertexMorphing.ControlSize + RigidBody.ControlSize

        self.Control = np.zeros(self.ControlSize)

        self.scaling_type = settings["scaling"]

    def Calculate(self):

        self.MappingMatrix = np.zeros([len(self.Design.Nodes),
                                       self.ControlSize])

        self.VertexMorphing.Calculate()

        computed_parameters = 0
        self.MappingMatrix[:,
                           computed_parameters:computed_parameters+self.VertexMorphing.ControlSize] = self.VertexMorphing.MappingMatrix

        computed_parameters += self.VertexMorphing.ControlSize
        self.RigidBody.Calculate()
        self.MappingMatrix[:,
                           computed_parameters:computed_parameters+self.RigidBody.ControlSize] = self.RigidBody.MappingMatrix

        self.scaling_matrix = np.eye(self.ControlSize)

        if self.scaling_type == "shape" or self.scaling_type == "shape_off":
            nodal_areas = self.Design.ComputeNodalAreas()
            mass_matrix = np.zeros((self.ControlSize, self.ControlSize))
            computed_parameters = 0
            mass_matrix[computed_parameters:computed_parameters+self.VertexMorphing.ControlSize,
                                computed_parameters:computed_parameters+self.VertexMorphing.ControlSize] = self.VertexMorphing.CalculateDiagonalMassMatrix(nodal_areas, self.VertexMorphing.MappingMatrix)
            computed_parameters += self.VertexMorphing.ControlSize

            if self.scaling_type == "shape_off":
                mass_matrix_off_diag = self.CalculateMassMatrixOffDiagonal(nodal_areas, self.VertexMorphing.MappingMatrix, self.RigidBody.MappingMatrix)
                mass_matrix[0:self.VertexMorphing.ControlSize,
                            computed_parameters:computed_parameters+self.RigidBody.ControlSize] = mass_matrix_off_diag

                mass_matrix[computed_parameters:computed_parameters+self.RigidBody.ControlSize,
                            0:self.VertexMorphing.ControlSize] = mass_matrix_off_diag.transpose()

            mass_matrix[computed_parameters:computed_parameters+self.RigidBody.ControlSize,
                        computed_parameters:computed_parameters+self.RigidBody.ControlSize] = self.CalculateMassMatrix(nodal_areas, self.RigidBody.MappingMatrix)

            # print("Mass Matrix: {}".format(mass_matrix))

            self.scaling_matrix = self.CalculateVariableScalingMatrix(mass_matrix)

        elif self.scaling_type == "shape_sub":
            self.scaling_matrix = np.zeros((self.ControlSize, self.ControlSize))
            computed_parameters = 0
            self.scaling_matrix[computed_parameters:computed_parameters+self.VertexMorphing.ControlSize,
                                computed_parameters:computed_parameters+self.VertexMorphing.ControlSize] = self.VertexMorphing.scaling_matrix

            computed_parameters += self.VertexMorphing.ControlSize

            self.scaling_matrix[computed_parameters:computed_parameters+self.RigidBody.ControlSize,
                                computed_parameters:computed_parameters+self.RigidBody.ControlSize] = self.RigidBody.scaling_matrix

        # self.scaling_matrix = np.eye(self.ControlSize)
        print("Scaling Matrix: {}".format(self.scaling_matrix))

        # exit()
        # print("Mapping Matrix: {}".format(self.MappingMatrix))
        # if self.scaling_type == "none":
        #     self.scaling_matrix = np.eye(self.ControlSize)
        # elif self.scaling_type == "column":
        #     self.scaling_matrix = np.zeros([self.ControlSize, self.ControlSize])
        #     for column in range(self.ControlSize):
        #         scale_factor = np.max(self.MappingMatrix[:, column])
        #         self.scaling_matrix[column, column] = 1/scale_factor
        # elif self.scaling_type == "shape":
        #     nodal_areas = self.Design.ComputeNodalAreas()
        #     mass_matrix = self.CalculateMassMatrix(nodal_areas, self.MappingMatrix)
        #     self.scaling_matrix = self.CalculateVariableScalingMatrix(mass_matrix)
        # else:
        #     ValueError("'scaling_type' unknown!")



    def MapGradient(self, gradient):

        if self.scaling_type == "none":
            mapped_gradient = self.MappingMatrix.transpose() @ gradient
        elif self.scaling_type == "column" or self.scaling_type in ["shape", "shape_sub", "shape_off"]:
            mapped_gradient = self.scaling_matrix.transpose() @ self.MappingMatrix.transpose() @ gradient
        elif self.scaling_type == "pseudo_inv":
            mapped_gradient = np.linalg.pinv(self.MappingMatrix @ self.scaling_matrix) @ gradient

        return mapped_gradient

    def MapUpdate(self, control_update):

        if self.scaling_type == "none":
            design_update = self.MappingMatrix @ control_update
        elif self.scaling_type == "column" or self.scaling_type in ["shape", "shape_sub", "shape_off"]:
            design_update = self.MappingMatrix @ self.scaling_matrix @ control_update
        elif self.scaling_type == "pseudo_inv":
            design_update = self.MappingMatrix @ (self.scaling_matrix @ control_update)

        return design_update

    def GetUnscaledControlParameter(self, control_update):

        if self.scaling_type == "none" or self.scaling_type == "pseudo_inv":
            unscaled_control_update = control_update
        elif self.scaling_type == "column" or self.scaling_type in ["shape", "shape_sub", "shape_off"]:
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
        for i in range(matrix_a.shape[1]):
            for j in range(matrix_b.shape[1]):
                M[i, j] = matrix_a[:, i] @ np.multiply(matrix_b[:, j], nodal_areas)

        diagonal = M.diagonal()
        if np.any(diagonal == 0.0):
            M[diagonal == 0.0, diagonal == 0.0] = 1.0

        return M

    @staticmethod
    def CalculateMassMatrix(nodal_areas, matrix):
        M = np.zeros((matrix.shape[1], matrix.shape[1]))
        for i in range(matrix.shape[1]):
            for j in range(matrix.shape[1]):
                M[i, j] = matrix[:, i] @ np.multiply(matrix[:, j], nodal_areas)

        diagonal = M.diagonal()
        if np.any(diagonal == 0.0):
            M[diagonal == 0.0, diagonal == 0.0] = 1.0

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

