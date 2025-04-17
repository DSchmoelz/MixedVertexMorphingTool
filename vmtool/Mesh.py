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
            y += LinearNodeShapeFunction(x, node.x, shape_function_lengths[0], shape_function_lengths[1]) * node.z

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

    # TODO: Bessere Funktion Erstellen / Namen Ändern?
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

    def GetNodeInFilterRadius(self, x_i, r_left, r_right):

        nodes_in_filter = []

        for node in self.Nodes:
            if node.x > x_i - r_left and node.x < x_i + r_right:
                nodes_in_filter.append(node)

        return nodes_in_filter

    def ComputeNodalAreas(self):

        nodal_areas = np.zeros(len(self.Nodes))

        for node in self.Nodes:
            # neighbours = self.GetNodeNeighbours(node.id)
            # shape_function_lengths = self.GetNodeShapeFunctionLengths(node.id)

        #     if len(neighbours) == 1 and neighbours[0].x > node.x:
        #         delta_z = abs(neighbours[0].z - node.z)
        #         delta_x = shape_function_lengths[1]
        #         nodal_area_i = np.sqrt(delta_z**2 + delta_x**2)
        #     elif len(neighbours) == 1 and neighbours[0].x < node.x:
        #         delta_z = abs(node.z - neighbours[0].z)
        #         delta_x = shape_function_lengths[0]
        #         nodal_area_i = np.sqrt(delta_z**2 + delta_x**2)
        #     else:
        #         delta_z_0 = abs(node.z - neighbours[0].z)
        #         delta_x_0 = shape_function_lengths[0]
        #         delta_z_1 = abs(neighbours[1].z - node.z)
        #         delta_x_1 = shape_function_lengths[1]
        #         shape_function_lengths[1] = neighbours[1].x - node.x
        #         shape_function_lengths[0] = node.x - neighbours[0].x


        #     nodal_areas[self.GetNodeIndex(node.id)] = nodal_area_i
            nodal_areas[self.GetNodeIndex(node.id)] = np.sum(self.GetNodeShapeFunctionLengths(node.id))

        return nodal_areas

    def UpdateDesignVariables(self, design_update):

        for node in self.Nodes:
            node.z += design_update[self.GetNodeIndex(node.id)]

    def ComputeBlendingFunction(self, node_set, radius):

        blending_function = np.zeros(len(self.Nodes))
        for blending_node in node_set:
            nodes_in_filter = self.GetNodeInFilterRadius(blending_node.x, radius, radius)
            for node in nodes_in_filter:
                # if node.id in node_id_set:
                #     blending_value = 1
                # else:
                blending_value = LinearFilter(blending_node.x, node.x, radius)
                index = self.GetNodeIndex(node.id)
                if blending_value > blending_function[index]:
                    blending_function[index] = blending_value

        return blending_function
