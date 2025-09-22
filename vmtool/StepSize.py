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

    def ComputeStepSize(self, control_gradients=None, objective=None):
        pass

class ConstStepInControl(StepSize):

    def __init__(self, InitialStepSize):

        super().__init__()
        self.StepSize = InitialStepSize

    def ComputeStepSize(self, control_gradients=None, objective=None):
        return self.StepSize

class ConstStepInUnscaledControl(StepSize):

    def __init__(self, InitialStepSize, Mapper):

        super().__init__()
        self.StepSizeUnscaled = InitialStepSize
        self.Mapper = Mapper

    def ComputeStepSize(self, scaled_control_gradients, objective=None):
        scaled_step_size = self.StepSizeUnscaled / np.linalg.norm(self.Mapper.scaling_matrix @ scaled_control_gradients)

        return scaled_step_size

class GoldenSectionLineSearch(StepSize):
    # implementation according to "Multidiscipline Design Optimization", Vanderplaats
    def __init__(self, MaxStepSize, Tolerance, Mapper):

        super().__init__()
        self.MaxStepSize = MaxStepSize
        self.Mapper = Mapper

        self.tau = 0.381966 # golden section ratio

        self.NumberOfSteps = np.log(Tolerance) / np.log(1-self.tau) + 3

    def ComputeStepSize(self, search_direction, objective):

        def _CalculateObjective(alpha, search_direction, objective):

            control_update = - alpha * search_direction
            design_update = self.Mapper.MapUpdate(control_update)
            self.Mapper.Design.UpdateDesignVariables(design_update)
            objective.Calculate()
            f = objective.Value
            self.Mapper.Design.UpdateDesignVariables(-design_update)

            return f
        # print("### Step Size Computation start ###")
        alpha_l = 0
        alpha_u = self.MaxStepSize

        alpha_1 = (1-self.tau)*alpha_l + self.tau*alpha_u
        f_1 = _CalculateObjective(alpha_1, search_direction, objective)

        alpha_2 = self.tau*alpha_l + (1-self.tau)*alpha_u
        f_2 = _CalculateObjective(alpha_2, search_direction, objective)
        # print(f"### Max Number of Steps: {self.NumberOfSteps}")
        K = 3 + 1
        while K < self.NumberOfSteps:
            # print(f"### Step: {K}")
            # print(f"### alpha_l: {alpha_l}")
            # print(f"### alpha_u: {alpha_u}")
            # print(f"### alpha_1: {alpha_1}")
            # print(f"### alpha_2: {alpha_2}")
            # print(f"### f_1: {f_1}")
            # print(f"### f_2: {f_2}")
            if f_1 > f_2:
                alpha_l = alpha_1
                f_l = f_1

                alpha_1 = alpha_2
                f_1 = f_2

                alpha_2 = self.tau*alpha_l + (1-self.tau)*alpha_u
                f_2 = _CalculateObjective(alpha_2, search_direction, objective)

            else:
                alpha_u = alpha_2
                f_u = f_2

                alpha_2 = alpha_1
                f_2 = f_1

                alpha_1 = (1-self.tau)*alpha_l + self.tau*alpha_u
                f_1 = _CalculateObjective(alpha_1, search_direction, objective)

            K += 1

        if alpha_l == 0.0:
            return alpha_1

        return alpha_l
