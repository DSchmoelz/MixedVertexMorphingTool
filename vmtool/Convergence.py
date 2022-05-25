#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Convergence Method
#####################################################################

# external imports
import numpy as np

class Convergence(object):

    def __init__(self):
        self.Converged = False

    def CheckIfConverged(self):
        pass

class MaxSteps(Convergence):

    def __init__(self, MaxAmountOfSteps):

        super().__init__()
        self.MaxSteps = MaxAmountOfSteps

    def CheckIfConverged(self, StepNumber):
        if StepNumber >= self.MaxSteps:
            self.Converged = True