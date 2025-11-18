#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Rigid Body Parameterization
#####################################################################

# external imports
import numpy as np
import time
# internal imports
from .Node import *
from .Mesh import Mesh

class RigidBodyParameterization():

    def __init__(self, DesignMesh, settings):

        """
        settings: {
            "translation": True,
            "rotation": True,
            "scaling": "none"
        }
        """
        self.Design = DesignMesh
        # control is numpy array of parameter [Translation, Rotation]
        self.ControlSize = 0
        self.translation = settings["translation"]
        self.rotation = settings["rotation"]
        self.scaling_type = settings["scaling"]
        if self.translation is True:
            self.ControlSize += 1
        if self.rotation is True:
            self.ControlSize += 1

        self.center = None
        if "center" in settings:
            self.center = np.array(settings["center"])

        self.Control = np.zeros(self.ControlSize)

        self.scaling_calculation_time = []

    def Calculate(self, blending=None):

        self.MappingMatrix = np.zeros([len(self.Design.Nodes),
                                       self.ControlSize])

        computed_parameters = 0
        if self.translation:
            self.MappingMatrix[:, computed_parameters:computed_parameters+1] = self.ComputeTranslationMapping()
            computed_parameters += 1
        if self.rotation:
            self.MappingMatrix[:, computed_parameters:computed_parameters+1] = self.ComputeRotationMapping()

        if blending is not None:
            self.MappingMatrix *= blending[:, np.newaxis]

        start = time.time()
        if self.scaling_type == "none":
            self.scaling_matrix = np.eye(self.ControlSize)
        elif self.scaling_type == "column":
            self.scaling_matrix = np.zeros([self.ControlSize, self.ControlSize])
            for column in range(self.ControlSize):
                scale_factor = np.max(self.MappingMatrix[:, column])
                self.scaling_matrix[column, column] = 1/scale_factor
        elif self.scaling_type == "shape":
            nodal_areas = self.Design.ComputeNodalAreas()
            mass_matrix = self.CalculateMassMatrix(nodal_areas, self.MappingMatrix)
            self.scaling_matrix = self.CalculateVariableScalingMatrix(mass_matrix)
        elif self.scaling_type == "shape_diag_mass":
            nodal_areas = self.Design.ComputeNodalAreas()
            mass_matrix = self.CalculateDiagonalMassMatrix(nodal_areas, self.MappingMatrix)
            self.scaling_matrix = np.zeros((self.ControlSize, self.ControlSize))
            np.fill_diagonal(self.scaling_matrix, np.sqrt(np.reciprocal(np.diag(mass_matrix))))
        elif self.scaling_type == "sens_shape":
            nodal_areas = self.Design.ComputeNodalAreas()
            mass_matrix = self.CalculateMassMatrix(nodal_areas, self.MappingMatrix)
            self.scaling_matrix = np.linalg.inv(mass_matrix)
        elif self.scaling_type == "sens_shape_diag_mass":
            nodal_areas = self.Design.ComputeNodalAreas()
            mass_matrix = self.CalculateDiagonalMassMatrix(nodal_areas, self.MappingMatrix)
            self.scaling_matrix = np.linalg.inv(mass_matrix)
        else:
            ValueError("'scaling_type' unknown!")

        end = time.time()
        self.scaling_calculation_time.append(end - start)

    def MapGradient(self, gradient):

        if self.scaling_type == "none":
            mapped_gradient = self.MappingMatrix.transpose() @ gradient
        elif self.scaling_type in ["column", "shape", "shape_diag_mass", "sens_shape", "sens_shape_diag_mass"]:
            mapped_gradient = self.scaling_matrix.transpose() @ self.MappingMatrix.transpose() @ gradient

        return mapped_gradient

    def MapUpdate(self, control_update):

        if self.scaling_type in ["none", "sens_shape", "sens_shape_diag_mass"]:
            design_update = self.MappingMatrix @ control_update
        elif self.scaling_type in ["column", "shape", "shape_diag_mass"]:
            design_update = self.MappingMatrix @ self.scaling_matrix @ control_update

        return design_update

    def GetUnscaledControlParameter(self, control_update):

        if self.scaling_type in ["none", "sens_shape", "sens_shape_diag_mass"]:
            unscaled_control_update = control_update
        elif self.scaling_type in ["column", "shape", "shape_diag_mass"]:
            unscaled_control_update = self.scaling_matrix @ control_update

        return unscaled_control_update

    def ComputeTranslationMapping(self):

        return np.ones([len(self.Design.Nodes), 1])

    def ComputeRotationMapping(self):

        center_of_gravity = self.GetCenterOfGravity()
        A = np.zeros([len(self.Design.Nodes), 1])

        for design_node in self.Design.Nodes:
            design_node_index = self.Design.GetNodeIndex(design_node.id)

            A[design_node_index, 0] = design_node.x - center_of_gravity

        return A

    def GetCenterOfGravity(self):

        if self.center:
            return self.center

        nodal_areas = self.Design.ComputeNodalAreas()
        node_coordinates = self.Design.GetNodeCoordinatesX()

        center_of_gravity = np.dot(nodal_areas, node_coordinates) / np.sum(nodal_areas)

        return center_of_gravity

    @staticmethod
    def CalculateMassMatrix(nodal_areas, matrix):
        M = np.zeros((matrix.shape[1], matrix.shape[1]))
        for i in range(matrix.shape[1]):
            for j in range(matrix.shape[1]):
                M[i, j] = matrix[:, i] @ np.multiply(matrix[:, j], nodal_areas)

        return M

    @staticmethod
    def CalculateDiagonalMassMatrix(nodal_areas, matrix):
        M = np.zeros((matrix.shape[1], matrix.shape[1]))
        for i in range(matrix.shape[1]):
            M[i, i] = np.sum(np.multiply(abs(matrix[:, i]), nodal_areas))

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
