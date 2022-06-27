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

    def ComputeStepSize(self, control_gradients=None):
        pass

class ConstStepInControl(StepSize):

    def __init__(self, InitialStepSize):

        super().__init__()
        self.StepSize = InitialStepSize

    def ComputeStepSize(self, control_gradients=None):
        return self.StepSize

class ConstStepInUnscaledControl(StepSize):

    def __init__(self, InitialStepSize, Mapper):

        super().__init__()
        self.StepSizeUnscaled = InitialStepSize
        self.Mapper = Mapper

    def ComputeStepSize(self, scaled_control_gradients):
        scaled_step_size = self.StepSizeUnscaled / np.linalg.norm(self.Mapper.scaling_matrix @ scaled_control_gradients)

        return scaled_step_size
