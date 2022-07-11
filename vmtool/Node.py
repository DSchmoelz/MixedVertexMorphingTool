#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Node
#####################################################################

class Node(object):

    def __init__(self, id, x):
        self.id = id
        self.x = x

class DesignNode(Node):

    def __init__(self, id, x, z):
        super().__init__(id, x)
        self.z = z

# TODO: ControlNodes volltändig löschen?
class ControlNode(Node):

    def __init__(self, id, x, z):
        super().__init__(id, x)
        self.z = z
        self.zeta = 0