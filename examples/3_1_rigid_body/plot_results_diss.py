import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import shutil
from mixedvmtool import *
from dissertation_plots.plots import Plot, SubPlots
from dissertation_plots.tum_colors import *
from run_rigid_body import CreateTargetMesh, CreateDesignMesh, target_geometry

def create_objective_meshgrid(settings, target_geometry):

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

    if "center" in settings:
        rigid_body_settings = {
            "translation": True,
            "rotation": True,
            "scaling": "none",
            "center": settings["center"]
        }
    else:
        rigid_body_settings = {
            "translation": True,
            "rotation": True,
            "scaling": "none"
        }
    Mapper = RigidBodyParameterization("dummy", rigid_body_settings)

    t_min = settings["translation_interval"][0]
    t_max = settings["translation_interval"][1]
    translation = np.linspace(t_min, t_max)
    r_min = settings["rotation_interval"][0]
    r_max = settings["rotation_interval"][1]
    rotation = np.linspace(r_min, r_max)
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

    return translation, rotation, f

result_folder = "references/ref_results"
figure_folder = "figures"
if os.path.exists(f"{figure_folder}"):
    shutil.rmtree(f"{figure_folder}")
os.makedirs(f"{figure_folder}")

histories = [
    os.path.join(result_folder, "history_scaling_none"),
    os.path.join(result_folder, "history_scaling_shape_diag_mass"),
    os.path.join(result_folder, "history_scaling_shape")
]

### Objective plot
dfs = []

for history in histories:
    path = f"{history}/obj_history.csv"
    df = pd.read_csv(path, delimiter=",")
    df.columns = [x.strip() for x in df.columns]
    dfs.append(df)

plot = Plot(xlabel="iteration", ylabel="objective", height=4.5, width=6)
plot.ax.plot(dfs[0]["objective"], color=TUM_GRAY_2, label=f"unscaled")
plot.ax.plot(dfs[0]["objective"][-1:], color=TUM_GRAY_2, marker="x")
plot.ax.plot(dfs[1]["objective"], color=TUM_ORANGE, label=f"scaled diagonal", linestyle=":")
plot.ax.plot(dfs[1]["objective"][-1:], color=TUM_ORANGE, marker="x")
plot.ax.plot(dfs[2]["objective"], color=TUM_BLUE, label=f"scaled full")
plot.ax.plot(dfs[2]["objective"][-1:], color=TUM_BLUE, marker="x")
plot.ax.set_xticks([0, 10, 20, 30, 40])
plot.add_legend()
plt.tight_layout(pad=0.2)
plt.savefig(f"{figure_folder}/fig_rigid_body_obj.pdf")
plt.savefig(f"{figure_folder}/fig_rigid_body_obj.pgf")

### Optimization problem plot
TargetMesh = CreateTargetMesh(8)
InitialDesignMesh = CreateDesignMesh(8)
plot = Plot(xlabel=r'$\xi$', ylabel=r'$x$', height=4.5, width=4.5)
plot.ax.plot(InitialDesignMesh.GetNodeCoordinatesX(), InitialDesignMesh.GetShapeZ(), color=TUM_GRAY, label="initial", linewidth=0.75, marker="|", markersize=4)
plot.ax.plot(TargetMesh.GetNodeCoordinatesX(), TargetMesh.GetShapeZ(), color=TUM_BLUE, label="target")
plot.add_legend(loc="upper left")
plt.tight_layout(pad=0.2)
plt.savefig(f"{figure_folder}/fig_rigid_body_opt_problem.pdf")
plt.savefig(f"{figure_folder}/fig_rigid_body_opt_problem.pgf")

### Shape evolution
design_data = []
max_iterations = []
x = {}
z = {}
for i, history in enumerate(histories):
    iterations = len(dfs[i]["objective"]) - 1
    max_iterations.append(iterations)
    translation = [0]
    rotation = [0]
    x = []
    z = []
    for j in range(iterations):
        path = f"{history}/design_geometry/design_geometry_{j+1}.csv"
        df = pd.read_csv(path, delimiter=",")
        df.columns = [x.strip() for x in df.columns]
        translation.append(float(df["translation"][0]))
        rotation.append(float(df["rotation"][0]))
        x.append(df["x"])
        z.append(df["z"])
    data_dict = {
        "x": x,
        "z": z,
        "translation": translation,
        "rotation": rotation
        }
    design_data.append(data_dict)

labels = ["unscaled", "scaled diagonal", "scaled full"]
# plot_steps = [6, 4, 1]
number_of_plots = 5
for i in range(len(design_data)):
    plot = Plot(xlabel=r'$\xi$', ylabel=r'$x$', height=4.5, width=4.5)
    plot.ax.plot(InitialDesignMesh.GetNodeCoordinatesX(), InitialDesignMesh.GetShapeZ(), color=TUM_BLUE_5, label="initial", linewidth=0.75, marker="|", markersize=4)
    plot.ax.plot(TargetMesh.GetNodeCoordinatesX(), TargetMesh.GetShapeZ(), color=TUM_BLUE, label="target")
    iterations = len(dfs[i]["objective"]) - 1
    # number_of_plots = int(iterations/plot_steps[i])
    # plot_indices = np.linspace(0, iterations-1, number_of_plots, dtype=int)
    plot_indices = np.zeros(number_of_plots, dtype=int)
    plot_indices[1:] = np.geomspace(1, iterations-1, number_of_plots-1, dtype=int)
    if number_of_plots >= iterations:
        plot_indices = np.arange(iterations, dtype=int)
    color_map_values = np.linspace(0.2, 0.8, num=number_of_plots)
    color_map = plt.get_cmap('Greys')
    for j, index in enumerate(plot_indices):
        plot.ax.plot(design_data[i]["x"][index], design_data[i]["z"][index],
                     color=color_map(color_map_values[j]),
                     linewidth=0.75,
                     marker='.', markersize=2,
                     label="iteration {}".format(index+1))
    plot.add_legend(loc="lower right")
    plt.tight_layout(pad=0.2)
    plt.savefig(f"{figure_folder}/fig_rigid_body_shape_evolution_{labels[i].replace(" ", "_")}.pdf")
    plt.savefig(f"{figure_folder}/fig_rigid_body_shape_evolution_{labels[i].replace(" ", "_")}.pgf")

### 3D Objective plot
obj_plot_settings = {
        "figsize": [10.0, 7.0],
        "translation_interval": [0, 8],
        "rotation_interval": [-0.4, 1.4]
    }
t, r, f = create_objective_meshgrid(obj_plot_settings, target_geometry)
plot = Plot(height=4.5, width=9.0)
levels = np.linspace(10, 410, 9)

color_map = plt.get_cmap('Greys_r')
cs = plot.ax.contour(t, r, f, cmap=color_map, levels=levels, linewidths=0.75)
plot.ax.clabel(cs)
plot.ax.set(xlabel="translation", ylabel="rotation")
plot.ax.plot(design_data[0]["translation"], design_data[0]["rotation"], color=TUM_GRAY_2, label=f"unscaled")
plot.ax.plot(design_data[1]["translation"], design_data[1]["rotation"],color=TUM_ORANGE, label=f"scaled diagonal", linestyle=":")
plot.ax.plot(design_data[2]["translation"], design_data[2]["rotation"],color=TUM_BLUE, label=f"scaled full")
plot.add_legend()
plt.tight_layout(pad=0.2)
plt.savefig(f"{figure_folder}/fig_rigid_body_obj_contour.pdf")
plt.savefig(f"{figure_folder}/fig_rigid_body_obj_contour.pgf")
