#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Least-Square Response
#####################################################################

# external imports
import numpy as np
# internal imports
from .Node import *
from .Mesh import Mesh

class TargetGeometryResponse():

    # Response which yields the difference between the design geometry and a target geometry.
    # Nodal response value: g_i = z_i_target - z_i
    # Aggregation of the nodal values by square sum:
    # g = sum(g_i²) = sum[(z_i_target - z_i)²]
    def __init__(self, DesignMesh, TargetMesh):

        self.Design = DesignMesh
        self.Target = TargetMesh
        self.Gradients = np.zeros(len(DesignMesh.Nodes))
        self.Value = 0

    def Calculate(self):

        self.Value = 0

        for design_node in self.Design.Nodes:

            # g = z_i_target - z_i
            g_i =  self.Target.GetGeometryAt(design_node.x) - design_node.z
            # g = sum(g_i²)
            self.Value += g_i * g_i
            # dg/dz_i = 2 * (z_i_target - z_i) = -2 * g_i
            self.Gradients[self.Design.GetNodeIndex(design_node.id)] = -2 * g_i