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
# internal imports
from .Node import *
from .ShapeFunctions import *

class Mesh(object):

    def __init__(self, name):

        self.Name = name
        self.Nodes = []

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
        # TODO: Funktion funktioniert derzeit nur für "ControlMeshes", da node.p verwendet wird und nicht node.z
        for node in self.Nodes:
            shape_function_lengths = self.GetNodeShapeFunctionLengths(node.id)
            y += LinearNodeShapeFunction(x, node.x, shape_function_lengths[0], shape_function_lengths[1]) * node.p

        return y

    def GetShapeFunctionValueOfNode(self, x, node):

        shape_function_lengths = self.GetNodeShapeFunctionLengths(node.id)
        y = LinearNodeShapeFunction(x, node.x, shape_function_lengths[0], shape_function_lengths[1])

        return y

    def GetNodeCoordinatesX(self):

        nodes_x = []

        for node in self.Nodes:
            nodes_x.append(node.x)

        return np.array(nodes_x)

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

    def GetNodeShapeFunctionLengths(self, node_id):

        node = self.GetNodeWithId(node_id)
        shape_function_lengths = np.zeros(2)

        neighbours = self.GetNodeNeighbours(node_id)

        if len(neighbours) == 1 and neighbours[0].x > node.x:
            shape_function_lengths[1] = neighbours[0].x - node.x
        elif len(neighbours) == 1 and neighbours[0].x < node.x:
            shape_function_lengths[0] = node.x - neighbours[0].x
        else:
            shape_function_lengths[1] = neighbours[1].x - node.x
            shape_function_lengths[0] = node.x - neighbours[0].x

        return shape_function_lengths

    def GetNodeSpan(self, node_id):

        node = self.GetNodeWithId(node_id)
        node_shape_function_lengths = self.GetNodeShapeFunctionLengths(node_id)

        start = node.x - node_shape_function_lengths[0]
        end = node.x + node_shape_function_lengths[1]
        node_span = np.array([start, end])

        return node_span

    def GetNodeNeighbours(self, node_id):

        node = self.GetNodeWithId(node_id)
        nodes_x = self.GetNodeCoordinatesX()

        absolute_distance = np.abs(nodes_x - node.x)
        # set absolute distance of input node from 0 to 1e16
        node_index = self.GetNodeIndex(node_id)
        absolute_distance[node_index] = 1e16

        closest_node_index = absolute_distance.argmin()

        node1 = self.Nodes[closest_node_index]
        # node is start or end of geometry space => only one neighbour
        if node.x == self.Space()[0] or node.x == self.Space()[1]:
            neighbour = [node1]
            return neighbour

        else:
            # add 1e16 to the distance of the already found node
            absolute_distance[closest_node_index] += 1e16
            if node1.x < node.x:
                for nodej in self.Nodes:
                    if nodej.x < node.x:
                        indexj = self.GetNodeIndex(nodej.id)
                        absolute_distance[indexj] += 1e16

            elif node1.x > node.x:
                for nodej in self.Nodes:
                    if nodej.x > node.x:
                        indexj = self.GetNodeIndex(nodej.id)
                        absolute_distance[indexj] += 1e16

            closest_node_index = absolute_distance.argmin()
            node2 = self.Nodes[closest_node_index]

            if node1.x < node2.x:
                neighbours = [node1, node2]
            else:
                neighbours = [node2, node1]
            return neighbours

    def GetNodeInFilterRadius(self, x_i, r_left, r_right):

        nodes_in_filter = []

        for node in self.Nodes:
            if node.x >= x_i - r_left and node.x <= x_i + r_right:
                nodes_in_filter.append(node)

        return nodes_in_filter

    def ComputeNodalAreas(self):

        nodal_areas = np.zeros(len(self.Nodes))

        for node in self.Nodes:
            nodal_areas[self.GetNodeIndex(node.id)] = np.sum(self.GetNodeShapeFunctionLengths(node.id))

        return nodal_areas

