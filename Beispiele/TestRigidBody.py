#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# TestRigidBody
#####################################################################

import numpy as np
import sys
import path_setting
sys.path.append(path_setting.path)
from vmtool import *


from RigidBodyObjectivePlot import create_objective_plot

def target_geometry(x_j):
    p_j = x_j / 2 + 4

    return p_j

def CreateTargetMesh(x_limit):
    ## Target Geometry
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

    return TargetMesh

def CreateDesignMesh(x_limit):
    ## Design Geometry
    design_number_of_nodes = 2*(x_limit)+1
    x_i = np.linspace(-x_limit, x_limit, design_number_of_nodes)
    DesignNodeList = []
    design_ids = np.arange(design_number_of_nodes)
    for i in range(0, design_number_of_nodes):
        DesignNodeList.append(DesignNode(design_ids[i], x_i[i], 0))

    DesignMesh = Mesh("design")
    DesignMesh.AddNodes(DesignNodeList)

    return DesignMesh

def main():
    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator
    plt.style.use('seaborn-v0_8-paper')
    SMALL_SIZE = 8
    MEDIUM_SIZE = 10
    BIGGER_SIZE = 12

    plt.rc('font', size=BIGGER_SIZE)          # controls default text sizes
    plt.rc('axes', titlesize=BIGGER_SIZE)     # fontsize of the axes title
    plt.rc('axes', labelsize=BIGGER_SIZE)    # fontsize of the x and y labels
    plt.rc('xtick', labelsize=MEDIUM_SIZE)    # fontsize of the tick labels
    plt.rc('ytick', labelsize=MEDIUM_SIZE)    # fontsize of the tick labels
    plt.rc('legend', fontsize=BIGGER_SIZE)    # legend fontsize
    plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title
    # stuff only to run when not called via 'import' here
    x_limit = 8
    TargetMesh = CreateTargetMesh(x_limit)
    # all scaling types
    # scaling_types =["none", "column", "shape", "shape_diag_mass", "sens_shape", "sens_shape_diag_mass"]
    # colors = ['gray', 'green', 'red', 'orange', 'blue', 'cyan']
    # markers = ['X', 'P', 'o', 's', 'v', '<']
    # linestyles = ['solid', 'solid', 'solid', 'solid', (0, (5, 10)), (0, (5, 10))]

    # all scaling types beside pure sensitivity scaling
    scaling_types =["none", "shape", "shape_diag_mass"]
    colors = ['gray', 'red', 'orange']
    markers = ['X', 'o', 's']
    plot_steps = [5, 1, 3]
    linestyles = ['solid', 'solid', 'solid']

    # no scaling at all
    # scaling_types = ["none"]
    # colors = ['gray']
    # markers = ['X']
    # plot_steps = [5]
    # linestyles = ['solid']

    style = dict(linewidth=1.0)

    obj_plot_settings = {
        "figsize": [10.0, 7.0],
        "translation_interval": [0, 8],
        "rotation_interval": [-0.4, 1.4]
    }
    fig, ax = create_objective_plot(obj_plot_settings, target_geometry)
    fig.tight_layout()
    ax.view_init(elev=40., azim=15)
    figure_2D, axis_2D = plt.subplots(2, 2, figsize=[12.0,8.0])
    figure_2D.tight_layout(pad=2.0)

    for scaling_type, color, marker, linestyle, plot_step in zip(scaling_types, colors, markers, linestyles, plot_steps):

        DesignMesh = CreateDesignMesh(x_limit)

        figure_shape, axis_shape = plt.subplots(1, figsize=[5.0,5.0])
        axis_shape.plot(TargetMesh.GetNodeCoordinatesX(), TargetMesh.GetShapeZ(), label='target')
        axis_shape.plot(DesignMesh.GetNodeCoordinatesX(), DesignMesh.GetShapeZ(), color='lightskyblue', marker='o', markersize=5.0, markerfacecolor='lightskyblue', label='initial')
        axis_shape.plot(0, DesignMesh.GetGeometryAt(0), marker='o', markersize=10.0, markerfacecolor='lightsteelblue')
        axis_shape.set_xlabel(xlabel="local coordinate " +  r'$\xi$')
        axis_shape.axis('equal')
        axis_shape.legend()
        figure_shape.tight_layout()

        figure_shape.savefig("Plots/RigidBody/optproblem.png", dpi=600)

        ## Optimization Set-Up
        Response = TargetGeometryResponse("target", DesignMesh, TargetMesh)

        rigid_body_settings = {
            "translation": True,
            "rotation": True,
            "scaling": scaling_type
        }
        Mapper = RigidBodyParameterization(DesignMesh, rigid_body_settings)

        # StepSizeSettings = ConstStepInUnscaledControl(0.5, Mapper)
        StepSizeSettings = GoldenSectionLineSearch(100.0, 1e-8, Mapper)
        # StepSizeSettings = ConstStepInControl(1.0)

        max_steps = 50
        objective_value = 1e-6
        # ConvergenceSettings = MaxSteps(max_steps)
        ConvergenceSettings = ObjectiveValue(objective_value, max_steps)

        history_folder = f"rigid_body/history_scaling_{scaling_type}"
        OptimizationAlgorithm = SteepestDescentAlgorithm("Optimierung", Mapper, ConvergenceSettings, StepSizeSettings, NormalizeObjGrad=False, HistoryFolder=history_folder)
        OptimizationAlgorithm.AddObjective(Response)

        ## Start Optimization
        OptimizationAlgorithm.StartOptimization()
        # print(OptimizationAlgorithm.Mapper.MappingMatrix)
        # for i in range(len(OptimizationAlgorithm.PreviousControlFields)):
        #     print(20*"-")
        #     print("optimization step {}".format(i+1))
        #     print("gradient {}".format(OptimizationAlgorithm.PreviousControlFields[i]["dg/dp"]))
        #     print("control update {}".format(OptimizationAlgorithm.PreviousControlFields[i]["delta_p"]))
        #     print("objective value {}".format(OptimizationAlgorithm.PreviousObjectiveValue[i]))

        print(40*"-")
        print("final objective value {}".format(OptimizationAlgorithm.PreviousObjectiveValue[-1]))
        final_step = OptimizationAlgorithm.StepNumber-1

        f = OptimizationAlgorithm.PreviousObjectiveValue
        translation = [0]
        rotation = [0]
        for i in range(final_step):
            translation.append(OptimizationAlgorithm.ControlParameter[2*i])
            rotation.append(OptimizationAlgorithm.ControlParameter[2*i+1])

        # ax.plot(translation, rotation, f, label=scaling_type)
        ax.plot(translation, rotation, f, color=color, label=scaling_type, linestyle=linestyle)
        ax.legend(title="scaling types")

        axis_2D[0,1].plot(translation, color=color, label=scaling_type, linestyle=linestyle, **style)
        axis_2D[0,1].scatter(final_step, translation[-1], color=color, marker=marker)
        axis_2D[0,1].set(xlabel="Iteration", ylabel="Translation ")
        axis_2D[0,1].xaxis.set_major_locator(MaxNLocator(integer=True))

        axis_2D[1,1].plot(rotation, color=color, linestyle=linestyle, **style)
        axis_2D[1,1].scatter(final_step, rotation[-1], color=color, marker=marker)
        axis_2D[1,1].set(xlabel="Iteration", ylabel="Rotation")
        axis_2D[1,1].xaxis.set_major_locator(MaxNLocator(integer=True))

        axis_2D[0,0].plot(f, color=color, label=scaling_type, linestyle=linestyle, **style)
        axis_2D[0,0].scatter(final_step, f[-1], color=color, marker=marker)
        axis_2D[0,0].set(xlabel="Iteration", ylabel="Objective")
        axis_2D[0,0].xaxis.set_major_locator(MaxNLocator(integer=True))
        axis_2D[0,0].legend(title="scaling types")

        axis_2D[1,0].plot(f, color=color, label=scaling_type, linestyle=linestyle, **style)
        axis_2D[1,0].scatter(final_step, f[-1], color=color, marker=marker)
        axis_2D[1,0].set(xlabel="Iteration", ylabel="Objective")
        axis_2D[1,0].xaxis.set_major_locator(MaxNLocator(integer=True))
        axis_2D[1,0].set_yscale('log')
        axis_2D[1,0].axhline(y=1e-6, color='magenta',linestyle=linestyle, **style)

        number_of_plots = int(len(OptimizationAlgorithm.PreviousDesignFields)/plot_step-1)
        color_map_values = np.linspace(0.2, 0.8, num=number_of_plots)
        color_map = matplotlib.cm.get_cmap('Greys')
        for i in range(number_of_plots):
            x = OptimizationAlgorithm.PreviousDesignFields[plot_step*i+1]["x"]
            z = OptimizationAlgorithm.PreviousDesignFields[plot_step*i+1]["z"]
            axis_shape.plot(x, z, color=color_map(color_map_values[i]), marker='o', markersize=5, label="iteration {}".format(plot_step*i+1))

        FinalShape = OptimizationAlgorithm.Mapper.Design
        axis_shape.plot(FinalShape.GetNodeCoordinatesX(), FinalShape.GetShapeZ(), color='black', marker='o', markersize=5, label="iteration {}".format(final_step))
        axis_shape.axis('equal')
        axis_shape.legend()
        figure_shape.savefig("Plots/RigidBody/shape_{}.png".format(scaling_type), dpi=600)

    fig.savefig("Plots/RigidBody/objective_plot.png", dpi=600)
    figure_2D.savefig("Plots/RigidBody/convergence_plot.png", dpi=600)
    plt.show()

if __name__ == "__main__":
    main()