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
from .ShapeFunctions import *
from .GaussianQuadrature import *
from .Node import *
from .Mesh import Mesh

class GaussianIntegration():

    def __init__(self, DesignMesh, ControlMesh, FilterRadius):

        self.Design = DesignMesh
        self.Control = ControlMesh
        self.FilterRadius = FilterRadius

        self.MappingMatrix = np.zeros([len(self.Design.Nodes), len(self.Control.Nodes)])
        self.nGP = 2
        self.ControlSpace = self.Control.Space()
        self.DesignSpace = self.Design.Space()

    def Calculate(self):
        for design_node in self.Design.Nodes:

            integration_intervals = self.GetIntegrationIntervals(design_node)

            for interval in integration_intervals:
                gauss_points, gauss_weights = CalculateGaussPointsAndWeights(self.nGP, interval[0], interval[1])
                interval_length = interval[1] - interval[0]

                for control_node in self.Control.Nodes:
                    for gauss_point, gauss_weight in zip(gauss_points, gauss_weights):

                        self.MappingMatrix[self.Design.GetNodeIndex(design_node.id), self.Control.GetNodeIndex(control_node.id)] += interval_length/2 * LinearHatFunction(gauss_point, design_node.x, self.FilterRadius) * self.Control.GetShapeFunctionValueOfNode(gauss_point, control_node) * gauss_weight

    'Funktioniert noch nicht! Filter Funktion muss noch angepasst werden.'
    def CalculateVaryingFilter(self):
        for design_node in self.Design.Nodes:

            integration_intervals = self.GetIntegrationIntervals(design_node)

            for interval in integration_intervals:
                gauss_points, gauss_weights = CalculateGaussPointsAndWeights(self.nGP, interval[0], interval[1])
                interval_length = interval[1] - interval[0]

                for control_node in self.Control.Nodes:
                    for gauss_point, gauss_weight in zip(gauss_points, gauss_weights):
                        self.MappingMatrix[self.Design.GetNodeIndex(design_node.id), self.Control.GetNodeIndex(control_node.id)] += interval_length/2 * VaryingLinearHatFunction(gauss_point, design_node.x, self.FilterRadius, self.DesignSpace, 1) * self.Control.GetShapeFunctionValueOfNode(gauss_point, control_node) * gauss_weight

    def GetIntegrationIntervals(self, node):

        # find nodes in filter
        nodes_in_filter = self.Control.GetNodeInFilterRadius(node.x, self.FilterRadius, self.FilterRadius)

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