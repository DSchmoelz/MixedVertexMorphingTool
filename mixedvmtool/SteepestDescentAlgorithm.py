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
import pandas as pd
import os
import shutil
import time
# internal imports
from .Node import *
from .Mesh import Mesh
from .VertexMorphingRigidBodyParameterization import VertexMorphingRigidBodyParameterization
from .RigidBodyParameterization import RigidBodyParameterization

class SteepestDescentAlgorithm(object):

    def __init__(self, name, Mapper, Convergence, StepSize, NormalizeObjGrad=False, HistoryFolder="history"):

        self.Name = name
        self.Mapper = Mapper
        self.Convergence = Convergence
        self.StepSize = StepSize
        self.StepNumber = 0
        self.Objectives = {}
        self.NormalizeObjGrad = NormalizeObjGrad
        self.DesignFields = {}
        self.ControlFields = {}

        self.PreviousDesignFields = []
        self.PreviousControlFields = []
        self.PreviousObjectiveValue = []

        self.HistoryFolder = HistoryFolder
        self.history_data = {
            "iteration": [],
            "objective": [],
            "step_size": [],
            # "scaling_calc_time": [],
        }

    def AddObjective(self, Response):

        self.Objectives[Response.Name] = Response
        self.DesignFields["d{}/dz".format(Response.Name)] = np.zeros(len(self.Mapper.Design.Nodes))
        self.ControlFields["d{}/dp".format(Response.Name)] = np.zeros(self.Mapper.ControlSize)

    def StartOptimization(self):

        self._InitializeHistory()

        while not self.Convergence.Converged:
            self.StepNumber += 1
            print("Starting Optimization Step: {} of {}".format(self.StepNumber, self.Name))
            # if self.StepNumber == 1:
            self._CalculateMapping()
            self._CalculateObjectives()
            if self.StepNumber == 1:
                self.Convergence.InitialObjectiveValue = self.history_data["objective"][0]
            self._MapObjectiveGradients()
            self._CalculateControlUpdate()
            self._MapControlUpdate()
            self._UpdateDesign()
            ###
            for objective in self.Objectives.values():
                objective.Calculate()
                objective_value = objective.Value
            ###
            self._SaveHistory()
            self._SaveDesign()
            self.Convergence.CheckIfConverged(objective_value, self.StepNumber)

        # Finalisierung
        self.StepNumber += 1
        self._CalculateObjectives()
        self.history_data["step_size"].append(None)
        # self.history_data["scaling_calc_time"].append(None)
        self._SaveHistory()

    def _CalculateMapping(self):
        self.Mapper.Calculate()

    def _CalculateObjectives(self):
        for objective in self.Objectives.values():
            objective.Calculate()
            self.DesignFields["d{}/dz".format(objective.Name)] = objective.Gradients

        self.PreviousObjectiveValue.append(objective.Value)
        self.history_data["objective"].append(objective.Value)

    def _MapObjectiveGradients(self):
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
        objective = list(self.Objectives.values())[0]
        start = time.time()
        step_size = self.StepSize.ComputeStepSize(control_gradients, objective)
        end = time.time()
        print(f"Step size computation time: {end - start}s")
        self.history_data["step_size"].append(step_size)
        control_update = - step_size * control_gradients
        self.ControlFields["delta_p"] = control_update

        unscaled_control_update = np.zeros(self.Mapper.ControlSize)
        unscaled_control_update = self.Mapper.GetUnscaledControlParameter(control_update)

        if isinstance(self.Mapper, VertexMorphingRigidBodyParameterization):
            self.Mapper.RigidBody.UpdateRigidBodyParameter(unscaled_control_update)

        elif isinstance(self.Mapper, RigidBodyParameterization):
            self.Mapper.UpdateRigidBodyParameter(unscaled_control_update)

    def _MapControlUpdate(self):

        control_update = self.ControlFields["delta_p"]
        design_update = self.Mapper.MapUpdate(control_update)
        self.DesignFields["delta_z"] = design_update

    def _UpdateDesign(self):
        self.PreviousControlFields.append(self.ControlFields.copy())

        design_update = self.DesignFields["delta_z"]
        self.DesignFields["x"] = self.Mapper.Design.GetNodeCoordinatesX()
        self.DesignFields["z"] = self.Mapper.Design.GetShapeZ()
        self.PreviousDesignFields.append(self.DesignFields.copy())
        self.Mapper.Design.UpdateDesignVariables(design_update)
        self.DesignFields["x"] = self.Mapper.Design.GetNodeCoordinatesX()
        self.DesignFields["z"] = self.Mapper.Design.GetShapeZ()

    def _InitializeHistory(self):
        if os.path.exists(f"./{self.HistoryFolder}"):
            shutil.rmtree(f"./{self.HistoryFolder}")

        os.makedirs(f"./{self.HistoryFolder}")
        os.makedirs(f"./{self.HistoryFolder}/design_geometry")

    def _SaveHistory(self):
        self.history_data["iteration"] = np.arange(self.StepNumber)
        # self.history_data["scaling_calc_time"].append(self.Mapper.scaling_calculation_time)

        dataframe = pd.DataFrame(self.history_data)
        dataframe.to_csv(f"./{self.HistoryFolder}/obj_history.csv", index=False)

    def _SaveDesign(self):

        design_data = {
            "x": self.DesignFields["x"],
            "z": self.DesignFields["z"]
        }

        if isinstance(self.Mapper, VertexMorphingRigidBodyParameterization):
            if self.Mapper.RigidBody.translation:
                design_data["translation"] = self.Mapper.RigidBody.Control[0]
            if self.Mapper.RigidBody.rotation:
                design_data["rotation"] = self.Mapper.RigidBody.Control[1]

        elif isinstance(self.Mapper, RigidBodyParameterization):
            if self.Mapper.translation:
                design_data["translation"] = self.Mapper.Control[0]
            if self.Mapper.rotation:
                design_data["rotation"] = self.Mapper.Control[1]

        dataframe = pd.DataFrame(design_data)
        dataframe.to_csv(f"./{self.HistoryFolder}/design_geometry/design_geometry_{self.StepNumber}.csv", index=False)
