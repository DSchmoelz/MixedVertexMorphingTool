#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# TestSteepestDescentAlgorithm
#####################################################################

import numpy as np
import sys
import path_setting
sys.path.append(path_setting.path)
from vmtool import *
import matplotlib.pyplot as plt
from matplotlib import cm


def create_objective_plot():

    def target_geometry(x_j):
        p_j = x_j / 2 + 4

        return p_j

    def create_design():
        ## Design Geometry
        filter_radius = 4
        x_limit = filter_radius + 4
        design_number_of_nodes = 2*(x_limit)+1
        x_i = np.linspace(-x_limit, x_limit, design_number_of_nodes)

        DesignNodeList = []
        design_ids = np.arange(design_number_of_nodes)
        for i in range(0, design_number_of_nodes):
            DesignNodeList.append(DesignNode(design_ids[i], x_i[i], 0))

        DesignMesh = Mesh("design")
        DesignMesh.AddNodes(DesignNodeList)

        return DesignMesh

    ## Target Geometry
    filter_radius = 4
    x_limit = filter_radius + 4
    target_number_of_nodes = 3
    x_j = np.linspace(-x_limit, x_limit, target_number_of_nodes)
    p_j = np.zeros(target_number_of_nodes)

    TargetNodeList = []
    target_ids = np.arange(target_number_of_nodes)
    for i in range(0, target_number_of_nodes):
        p_j[i] = target_geometry(x_j[i])
        TargetNodeList.append(ControlNode(target_ids[i], x_j[i], p_j[i]))

    TargetMesh = Mesh("target")
    TargetMesh.AddNodes(TargetNodeList)

    rigid_body_settings = {
        "translation": True,
        "rotation": True,
        "scaling": "none"
    }
    Mapper = RigidBodyParameterization("dummy", rigid_body_settings)

    translation = np.linspace(0, 8)
    rotation = np.linspace(-0.4, 1.4)
    translation, rotation = np.meshgrid(translation, rotation)

    def CalculateObjective(t, r, Mapper):
        Mapper.Design = create_design()
        Mapper.Calculate()
        design_update = Mapper.MappingMatrix @ np.array([t, r])
        Mapper.Design.UpdateDesignVariables(design_update)
        Response = TargetGeometryResponse("target", Mapper.Design, TargetMesh)
        Response.Calculate()
        return Response.Value

    vfunc = np.vectorize(CalculateObjective)
    f = vfunc(translation, rotation, Mapper)
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})


    ax.plot_surface(translation, rotation, f, cmap=cm.coolwarm,
                        linewidth=0, antialiased=False, alpha=0.5)
    ax.set(xlabel="translation", ylabel="rotation", zlabel="objective")

    return fig, ax

