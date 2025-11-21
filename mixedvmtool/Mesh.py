#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Mesh
#####################################################################

# external imports
import numpy as np
import time
# internal imports
from .Node import *
from .ShapeFunctions import *

class Mesh(object):

    def __init__(self, name):

        self.Name = name
        self.Nodes = []
        self.node_coordinates_x = None

    def AddNodes(self, nodes):

        existing_node_ids = self.GetNodeIds()
        for node in nodes:
            if node.id not in existing_node_ids:
                self.Nodes.append(node)
            else:
                raise RuntimeError("Node with id {} already exists!".format(node.id))

    def AddNode(self, node):

        existing_node_ids = self.GetNodeIds()
        if node.id not in existing_node_ids:
            self.Nodes.append(node)
        else:
            raise RuntimeError("Node with id {} already exists!".format(node.id))

    def Space(self):
        nodes_x = self.GetNodeCoordinatesX()
        start = nodes_x[np.argmin(nodes_x)]
        end = nodes_x[np.argmax(nodes_x)]

        return np.array([start, end])

    def GetGeometryAt(self, x):

        y = 0
        for node_index, node in enumerate(self.Nodes):
            shape_function_lengths = self.GetNodeShapeFunctionLengths(node_index, node)
            y += LinearNodeShapeFunction(x, node.x, shape_function_lengths[0], shape_function_lengths[1]) * node.z

        return y

    def GetNodeCoordinatesX(self):

        if self.node_coordinates_x is None:
            nodes_x = []

            for node in self.Nodes:
                nodes_x.append(node.x)

            self.node_coordinates_x = np.array(nodes_x)

        return self.node_coordinates_x

    def GetShapeZ(self):

        nodes_z = []

        for node in self.Nodes:
            nodes_z.append(node.z)

        return np.array(nodes_z)

    def GetNodeIds(self):
        node_ids = []

        for node in self.Nodes:
            node_ids.append(node.id)

        return node_ids

    def GetNodeWithId(self, node_id):

        for node in self.Nodes:
            if node_id == node.id:
                return node
        else:
            raise RuntimeError("Node with id {} not found!".format(node_id))

    def GetNodeIndex(self, node_id):

        for node in self.Nodes:
            if node.id == node_id:
                return self.Nodes.index(node)

    def GetNodeShapeFunctionLengths(self, node_index, node):

        shape_function_lengths = np.zeros(2)
        if node_index == 0:
            shape_function_lengths[1] = self.Nodes[1].x - node.x
        elif node_index == len(self.Nodes)-1:
            shape_function_lengths[0] = node.x - self.Nodes[-2].x
        else:
            shape_function_lengths[1] = self.Nodes[node_index+1].x - node.x
            shape_function_lengths[0] = node.x - self.Nodes[node_index-1].x

        return shape_function_lengths

    # def GetNodeShapeFunctionLengths(self, node_id):

    #     node = self.GetNodeWithId(node_id)
    #     shape_function_lengths = np.zeros(2)

    #     start = time.time()
    #     neighbours = self.GetNodeNeighbours(node_id)
    #     end = time.time()
    #     print(f"GetNodeShapeFunctionLengths: Neighbours found in {end - start}s")

    #     if len(neighbours) == 1 and neighbours[0].x > node.x:
    #         shape_function_lengths[1] = neighbours[0].x - node.x
    #     elif len(neighbours) == 1 and neighbours[0].x < node.x:
    #         shape_function_lengths[0] = node.x - neighbours[0].x
    #     else:
    #         shape_function_lengths[1] = neighbours[1].x - node.x
    #         shape_function_lengths[0] = node.x - neighbours[0].x

    #     return shape_function_lengths

    def GetNodeNeighbours(self, node_id):

        node = self.GetNodeWithId(node_id)
        nodes_x = self.GetNodeCoordinatesX()

        absolute_distance = np.abs(nodes_x - node.x)
        # set absolute distance of input node from 0 to float(1e8)
        node_index = self.GetNodeIndex(node_id)
        absolute_distance[node_index] = float(1e8)

        closest_node_index = absolute_distance.argmin()

        node1 = self.Nodes[closest_node_index]
        # node is start or end of geometry space => only one neighbour
        if node.x == self.Space()[0] or node.x == self.Space()[1]:
            neighbour = [node1]
            return neighbour

        else:
            # add float(1e8) to the distance of the already found node
            absolute_distance[closest_node_index] += float(1e8)
            if node1.x < node.x:
                for nodej in self.Nodes:
                    if nodej.x < node.x:
                        indexj = self.GetNodeIndex(nodej.id)
                        absolute_distance[indexj] += float(1e8)

            elif node1.x > node.x:
                for nodej in self.Nodes:
                    if nodej.x > node.x:
                        indexj = self.GetNodeIndex(nodej.id)
                        absolute_distance[indexj] += float(1e8)

            closest_node_index = absolute_distance.argmin()
            node2 = self.Nodes[closest_node_index]

            if node1.x < node2.x:
                neighbours = [node1, node2]
            else:
                neighbours = [node2, node1]
            return neighbours

    def GetNodeInFilterRadius(self, x_i, node_x, r_left, r_right):

        node_indices = np.where((node_x > x_i - r_left) & (node_x < x_i + r_right))[0]

        return node_indices

    def ComputeNodalAreas(self):

        nodal_areas = np.zeros(len(self.Nodes))

        for node_index, node in enumerate(self.Nodes):
            nodal_areas[node_index] = np.sum(self.GetNodeShapeFunctionLengths(node_index, node))

        return nodal_areas

    def UpdateDesignVariables(self, design_update):

        for node in self.Nodes:
            node.z += design_update[self.GetNodeIndex(node.id)]

    def ComputeBlendingFunction(self, node_set, radius):

        blending_function = np.zeros(len(self.Nodes))
        node_x = self.GetNodeCoordinatesX()
        for blending_node in node_set:
            nodes_indices_in_filter = self.GetNodeInFilterRadius(blending_node.x, node_x, radius, radius)
            for node_index in nodes_indices_in_filter:
                # if node.id in node_id_set:
                #     blending_value = 1
                # else:
                blending_value = LinearFilter(blending_node.x, node_x[node_index], radius)
                # index = self.GetNodeIndex(node.id)

                if blending_value > blending_function[node_index]:
                    blending_function[node_index] = blending_value

        return blending_function
