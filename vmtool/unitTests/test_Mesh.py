#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Mesh Test
#####################################################################

import numpy as np
import unittest

import sys
import path_setting
sys.path.append(path_setting.path)
from vmtool import *

class TestMesh(unittest.TestCase):

    def initialize_node_list(self):
        x_i = np.array([0, 1, 2, 3, 4])

        p_i = np.array([0, 3, 2, 4, 1])

        ids = np.array([1, 4, 0, 2, 3])

        NodeList = []
        for i in range(0, len(x_i)):
            NodeList.append(ControlNode(ids[i], x_i[i], p_i[i]))

        return NodeList

    def test_add_nodes(self):

        node_list = self.initialize_node_list()

        mesh = Mesh("control")
        mesh.AddNodes(node_list)

        self.assertEqual(mesh.GetNodeWithId(0).x, 2)
        self.assertEqual(mesh.GetNodeWithId(1).x, 0)
        self.assertEqual(mesh.GetNodeWithId(2).x, 3)
        self.assertEqual(mesh.GetNodeWithId(3).x, 4)
        self.assertEqual(mesh.GetNodeWithId(4).x, 1)

        self.assertEqual(mesh.GetNodeWithId(0).p, 2)
        self.assertEqual(mesh.GetNodeWithId(1).p, 0)
        self.assertEqual(mesh.GetNodeWithId(2).p, 4)
        self.assertEqual(mesh.GetNodeWithId(3).p, 1)
        self.assertEqual(mesh.GetNodeWithId(4).p, 3)

        self.assertListEqual(mesh.GetNodeIds(), [1, 4, 0, 2, 3])

    def test_add_node(self):

        node_list = self.initialize_node_list()

        mesh = Mesh("control")
        mesh.AddNodes(node_list)

        new_node = ControlNode(5, 5, 10)

        mesh.AddNode(new_node)

        self.assertEqual(mesh.GetNodeWithId(5).x, 5)
        self.assertEqual(mesh.GetNodeWithId(5).p, 10)
        self.assertEqual(len(mesh.Nodes), 6)

    def test_get_node_neighbours(self):

        node_list = self.initialize_node_list()

        mesh = Mesh("control")
        mesh.AddNodes(node_list)

        self.assertListEqual(mesh.GetNodeNeighbours(0), [node_list[1], node_list[3]])
        self.assertListEqual(mesh.GetNodeNeighbours(1), [node_list[1]])
        self.assertListEqual(mesh.GetNodeNeighbours(2), [node_list[2], node_list[4]])
        self.assertListEqual(mesh.GetNodeNeighbours(3), [node_list[3]])
        self.assertListEqual(mesh.GetNodeNeighbours(4), [node_list[0], node_list[2]])

    def test_get_node_coordinates(self):

        node_list = self.initialize_node_list()

        mesh = Mesh("control")
        mesh.AddNodes(node_list)

        np.testing.assert_array_equal(mesh.GetNodeCoordinatesX(), np.array([0, 1, 2, 3, 4]))

    def test_get_space(self):

        node_list = self.initialize_node_list()

        mesh = Mesh("control")
        mesh.AddNodes(node_list)

        np.testing.assert_array_equal(mesh.Space(), np.array([0, 4]))

    def test_get_node_ids(self):

        node_list = self.initialize_node_list()

        mesh = Mesh("control")
        mesh.AddNodes(node_list)

        np.testing.assert_array_equal(mesh.GetNodeIds(), np.array([1, 4, 0, 2, 3]))

    def test_get_node_index(self):

        node_list = self.initialize_node_list()

        mesh = Mesh("control")
        mesh.AddNodes(node_list)

        np.testing.assert_array_equal(mesh.GetNodeIndex(0), 2)
        np.testing.assert_array_equal(mesh.GetNodeIndex(1), 0)
        np.testing.assert_array_equal(mesh.GetNodeIndex(2), 3)
        np.testing.assert_array_equal(mesh.GetNodeIndex(3), 4)
        np.testing.assert_array_equal(mesh.GetNodeIndex(4), 1)

    def test_get_geometry(self):

        node_list = self.initialize_node_list()

        mesh = Mesh("control")
        mesh.AddNodes(node_list)

        np.testing.assert_array_equal(mesh.GetGeometryAt(0), 0)
        np.testing.assert_array_equal(mesh.GetGeometryAt(0.5), 1.5)
        np.testing.assert_array_equal(mesh.GetGeometryAt(1.0), 3)
        np.testing.assert_array_equal(mesh.GetGeometryAt(1.5), 2.5)
        np.testing.assert_array_equal(mesh.GetGeometryAt(2.0), 2)
        np.testing.assert_array_equal(mesh.GetGeometryAt(2.5), 3)
        np.testing.assert_array_equal(mesh.GetGeometryAt(3.0), 4)
        np.testing.assert_array_equal(mesh.GetGeometryAt(3.5), 2.5)
        np.testing.assert_array_equal(mesh.GetGeometryAt(4.0), 1)


if __name__ == '__main__':
    unittest.main()