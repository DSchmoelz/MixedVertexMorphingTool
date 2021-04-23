#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Step Size Method
#####################################################################

# external imports
import numpy as np

class StepSize(object):

    def __init__(self):
        pass

class ConstStepInControl(StepSize):

    def __init__(self, InitialStepSize):

        super().__init__()
        self.StepSize = InitialStepSize