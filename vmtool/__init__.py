from .Node import *
from .Mesh import Mesh
from .ForwardMapping import *
from .RigidBodyParameterization import *
from .ShapeFunctions import *
from .TargetResponse import TargetGeometryResponse
from .SteepestDescentAlgorithm import SteepestDescentAlgorithm
from .StepSize import *
from .Convergence import *

def test():
    import unittest
    suite = unittest.TestLoader().discover('.', pattern='test*')
    unittest.TextTestRunner(verbosity=2).run(suite)
