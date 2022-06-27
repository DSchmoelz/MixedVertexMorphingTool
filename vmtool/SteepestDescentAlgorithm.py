#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# SteepestDescentAlgorithm
#####################################################################

# external imports
import numpy as np
# internal imports
from .Node import *
from .Mesh import Mesh

class SteepestDescentAlgorithm(object):

    def __init__(self, name, Mapper, Convergence, StepSize, NormalizeObjGrad=False):

        self.Name = name
        self.Mapper = Mapper
        self.Convergence = Convergence
        self.StepSize = StepSize
        self.StepNumber = 0
        self.Objectives = {}
        self.NormalizeObjGrad = NormalizeObjGrad
        self.DesignFields = {}
        self.ControlFields = {}
        # self.OldMapperList = []
        self.PreviousDesignFields = []
        self.PreviousControlFields = []
        self.PreviousObjectiveValue = []

        self.ControlParameter = []

    def AddObjective(self, Response):

        self.Objectives[Response.Name] = Response
        self.DesignFields["d{}/dz".format(Response.Name)] = np.zeros(len(self.Mapper.Design.Nodes))
        self.ControlFields["d{}/dp".format(Response.Name)] = np.zeros(self.Mapper.ControlSize)

    def StartOptimization(self):

        while not self.Convergence.Converged:
            self.StepNumber += 1
            self._CalculateMapping()
            self._CalculateObjectives()
            self._MapObjectiveGradients()
            self._CalculateControlUpdate()
            self._MapControlUpdate()
            self._UpdateDesign()
            self.Convergence.CheckIfConverged(self.StepNumber)

        # Finalisierung
        self._CalculateObjectives()

    def _CalculateMapping(self):
        self.Mapper.Calculate()

    def _CalculateObjectives(self):
        # TODO: Funktioniert derzeit nur für eine Response
        for objective in self.Objectives.values():
            objective.Calculate()
            self.DesignFields["d{}/dz".format(objective.Name)] = objective.Gradients

        self.PreviousObjectiveValue.append(objective.Value)

    def _MapObjectiveGradients(self):
        # TODO: Funktioniert derzeit nur für eine Response
        weight = 1
        self.ControlFields["dg/dp"] = np.zeros(self.Mapper.ControlSize)
        for objective in self.Objectives.values():
            mapped_gradients = self.Mapper.MapGradient(objective.Gradients)
            self.ControlFields["d{}/dp".format(objective.Name)] = mapped_gradients
            self.ControlFields["dg/dp"] += mapped_gradients * weight

        if self.NormalizeObjGrad:
            #max_norm = np.max(abs(self.ControlFields["dg/dp"]))
            l2_norm = np.linalg.norm(self.ControlFields["dg/dp"])
            self.ControlFields["dg/dp"] = (1/l2_norm) * self.ControlFields["dg/dp"]

    def _CalculateControlUpdate(self):

        control_gradients = self.ControlFields["dg/dp"]
        # delta_p = - alpha * dg/dp
        step_size = self.StepSize.ComputeStepSize(control_gradients)
        print("step_size: {}".format(step_size))
        control_update = - step_size * control_gradients
        self.ControlFields["delta_p"] = control_update

        unscaled_control_update = np.zeros(self.Mapper.ControlSize)
        unscaled_control_update = self.Mapper.GetUnscaledControlParameter(control_update)
        print("unscaled_control_update: {}".format(unscaled_control_update))

        for i in range(len(unscaled_control_update)):
            if self.StepNumber == 1:
                self.ControlParameter.append(unscaled_control_update[i])
            else:
                print(self.ControlParameter[-(len(unscaled_control_update)-i)])
                control_parameter = self.ControlParameter[-len(unscaled_control_update)] + unscaled_control_update[i]
                self.ControlParameter.append(control_parameter)

    def _MapControlUpdate(self):

        control_update = self.ControlFields["delta_p"]
        design_update = self.Mapper.MapUpdate(control_update)
        self.DesignFields["delta_z"] = design_update

    def _UpdateDesign(self):
        # save old geometry in old mapper
        # self.OldMapperList.append(self.Mapper)
        # save old design and control fields
        self.PreviousDesignFields.append(self.DesignFields.copy())
        self.PreviousControlFields.append(self.ControlFields.copy())

        design_update = self.DesignFields["delta_z"]
        self.Mapper.Design.UpdateDesignVariables(design_update)

    ## TODO: Plotten/Animieren der Ergebnisse. Ausserhalb des OptimierungsAlgorithmus?!
    ## Erstellen der DataFields usw. in einer eigenen Optimization Klasse??