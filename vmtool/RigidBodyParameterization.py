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
# internal imports
from .Node import *
from .Mesh import Mesh

class RigidBodyParameterization():

    def __init__(self, DesignMesh, Translation=True, Rotation=True):

        self.Design = DesignMesh
        # control is numpy array of parameter [Translation, Rotation]
        self.ControlSize = 0
        self.translation = Translation
        self.rotation = Rotation
        if self.translation is True:
            self.ControlSize += 1
        if self.rotation is True:
            self.ControlSize += 1

        self.Control = np.zeros(self.ControlSize)

    def Calculate(self):

        self.MappingMatrix = np.zeros([len(self.Design.Nodes),
                                       self.ControlSize])

        computed_parameters = 0
        if self.translation:
            self.MappingMatrix[:, computed_parameters:computed_parameters+1] = self.ComputeTranslationMapping()
            computed_parameters += 1
        if self.rotation:
            self.MappingMatrix[:, computed_parameters:computed_parameters+1] = self.ComputeRotationMapping()
            scale_factor = np.max(self.MappingMatrix[:, computed_parameters])
            self.MappingMatrix[:, computed_parameters] /= scale_factor


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

        nodal_areas = self.Design.ComputeNodalAreas()
        node_coordinates = self.Design.GetNodeCoordinatesX()

        center_of_gravity = np.dot(nodal_areas, node_coordinates) / np.sum(nodal_areas)

        return center_of_gravity

