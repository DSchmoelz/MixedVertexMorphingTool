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

    def CheckIfConverged(self, ObjectiveValue, StepNumber):
        pass

class MaxSteps(Convergence):

    def __init__(self, MaxAmountOfSteps):

        super().__init__()
        self.MaxSteps = MaxAmountOfSteps

    def CheckIfConverged(self, ObjectiveValue, StepNumber):
        if StepNumber >= self.MaxSteps:
            self.Converged = True

class ObjectiveValue(Convergence):

    def __init__(self, ObjectiveValue, MaxAmountOfSteps):

        super().__init__()
        self.ObjectiveValue = ObjectiveValue
        self.MaxSteps = MaxAmountOfSteps

    def CheckIfConverged(self, ObjectiveValue, StepNumber):

        if ObjectiveValue <= self.ObjectiveValue:
            self.Converged = True
        elif StepNumber >= self.MaxSteps:
            self.Converged = True
