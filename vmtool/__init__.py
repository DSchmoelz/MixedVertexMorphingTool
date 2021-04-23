from .Node import *
from .Mesh import Mesh
from .ForwardMapping import *
from .ShapeFunctions import *
from .TargetResponse import TargetGeometryResponse

def test():
    import unittest
    suite = unittest.TestLoader().discover('.', pattern='test*')
    unittest.TextTestRunner(verbosity=2).run(suite)
