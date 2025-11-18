import pandas as pd
import matplotlib.pyplot as plt
# import matplotlib.colormaps as cm
import numpy as np
import os
from plots import *
import logging
from pathlib import Path
import sys
import path_setting
sys.path.append(path_setting.path)
from vmtool import *
from TestRigidBody import CreateTargetMesh, CreateDesignMesh, target_geometry
from RigidBodyObjectiveMeshGrid import create_objective_meshgrid

logging.basicConfig(filename=f"{Path(__file__).name}.log", level=logging.INFO, filemode="w")

parent_dir = "/home/david/Software/VertexMorphingTool/VertexMorphingTool/Beispiele/"
sub_folder = "rigid_body"
dir = f"{parent_dir}{sub_folder}"

histories = [
    os.path.join(dir, "history_scaling_none"),
    os.path.join(dir, "history_scaling_shape_diag_mass"),
    os.path.join(dir, "history_scaling_shape")
]

### Objective plot
dfs = []

for history in histories:
    path = f"{history}/obj_history.csv"
    df = pd.read_csv(path, delimiter=",")
    df.columns = [x.strip() for x in df.columns]
    dfs.append(df)

plot = Plot(xlabel="iteration", ylabel="objective")
plot.ax.plot(dfs[0]["objective"], color=TUM_GRAY_2, label=f"unscaled")
plot.ax.plot(dfs[0]["objective"][-1:], color=TUM_GRAY_2, marker="x")
plot.ax.plot(dfs[1]["objective"], color=TUM_ORANGE, label=f"scaled diagonal", linestyle=":")
plot.ax.plot(dfs[1]["objective"][-1:], color=TUM_ORANGE, marker="x")
plot.ax.plot(dfs[2]["objective"], color=TUM_BLUE, label=f"scaled full")
plot.ax.plot(dfs[2]["objective"][-1:], color=TUM_BLUE, marker="x")

plot.add_legend()
plt.tight_layout(pad=0.2)
plt.show()
plt.savefig(f"figx_{sub_folder}_obj.pdf")

### Optimization problem plot
TargetMesh = CreateTargetMesh(8)
InitialDesignMesh = CreateDesignMesh(8)
plot = Plot()
plot.ax.plot(InitialDesignMesh.GetNodeCoordinatesX(), InitialDesignMesh.GetShapeZ(), color=TUM_GRAY, label="initial", linewidth=0.75, marker="|", markersize=4)
plot.ax.plot(TargetMesh.GetNodeCoordinatesX(), TargetMesh.GetShapeZ(), color=TUM_BLUE, label="target")
plot.add_legend(loc="upper left")
plt.tight_layout(pad=0.2)
plt.show()
plt.savefig(f"figx_{sub_folder}_opt_problem.pdf")

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
        path = f"{history}/design_geometry_{j+1}.csv"
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
    plot = Plot()
    plot.ax.plot(InitialDesignMesh.GetNodeCoordinatesX(), InitialDesignMesh.GetShapeZ(), color=TUM_BLUE_5, label="initial", linewidth=0.75, marker="|", markersize=4)
    plot.ax.plot(TargetMesh.GetNodeCoordinatesX(), TargetMesh.GetShapeZ(), color=TUM_BLUE, label="target")
    iterations = len(dfs[i]["objective"]) - 1
    # number_of_plots = int(iterations/plot_steps[i])
    # plot_indices = np.linspace(0, iterations-1, number_of_plots, dtype=int)
    plot_indices = np.zeros(number_of_plots, dtype=int)
    plot_indices[1:] = np.geomspace(1, iterations-1, number_of_plots-1, dtype=int)
    if number_of_plots >= iterations:
        plot_indices = np.arange(iterations, dtype=int)
    print(f"iterations: {iterations}")
    print(f"number_of_plots: {number_of_plots}")
    print(f"plot_indices: {plot_indices}")
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
    plt.show()
    plt.savefig(f"figx_{sub_folder}_shape_evolution_{labels[i].replace(" ", "_")}.pdf")

### 3D Objective plot
obj_plot_settings = {
        "figsize": [10.0, 7.0],
        "translation_interval": [0, 8],
        "rotation_interval": [-0.4, 1.4]
    }
t, r, f = create_objective_meshgrid(obj_plot_settings, target_geometry)
plot = Plot()
levels = np.linspace(10, 410, 9)
print(f"levels: {levels}")
color_map = plt.get_cmap('Greys_r')
cs = plot.ax.contour(t, r, f, cmap=color_map, levels=levels, linewidths=0.75)
plot.ax.clabel(cs)
plot.ax.set(xlabel="translation", ylabel="rotation")
plot.ax.plot(design_data[0]["translation"], design_data[0]["rotation"], color=TUM_GRAY_2, label=f"unscaled")
plot.ax.plot(design_data[1]["translation"], design_data[1]["rotation"],color=TUM_ORANGE, label=f"scaled diagonal", linestyle=":")
plot.ax.plot(design_data[2]["translation"], design_data[2]["rotation"],color=TUM_BLUE, label=f"scaled full")
plot.add_legend()
plt.tight_layout(pad=0.2)
plt.show()
plt.savefig(f"figx_{sub_folder}_obj_contour.pdf")


# # Translation plot
# design_data = []

# max_iterations = []
# for i, history in enumerate(histories):
#     iterations = len(dfs[i]["objective"]) - 1
#     max_iterations.append(iterations)
#     translation = [0]
#     rotation = [0]
#     for j in range(iterations):
#         path = f"{history}/design_geometry_{j+1}.csv"
#         df = pd.read_csv(path, delimiter=",")
#         df.columns = [x.strip() for x in df.columns]
#         translation.append(float(df["translation"][0]))
#         rotation.append(float(df["rotation"][0]))
#     data_dict = {
#         "translation": translation,
#         "rotation": rotation
#         }
#     design_data.append(data_dict)

# plot = Plot(xlabel="iteration", ylabel="translation")
# # plt.yscale('log')
# plt.xscale('symlog')
# plot.ax.plot(design_data[0]["translation"], color=TUM_GRAY_2, label=f"unscaled")
# plot.ax.plot(max_iterations[0], design_data[0]["translation"][-1:], color=TUM_GRAY_2, marker="x")
# plot.ax.plot(design_data[1]["translation"], color=TUM_ORANGE, label=f"scaled full diagonal", linestyle=":")
# plot.ax.plot(max_iterations[1], design_data[1]["translation"][-1:], color=TUM_ORANGE, marker="x")
# plot.ax.plot(design_data[2]["translation"], color=TUM_BLUE, label=f"scaled mixed")
# plot.ax.plot(max_iterations[2], design_data[2]["translation"][-1:], color=TUM_BLUE, marker="x")
# plot.ax.plot(design_data[3]["translation"], color=TUM_GRAY, label=f"scaled w off-diagonal", linestyle=":")
# plot.ax.plot(max_iterations[3], design_data[3]["translation"][-1:], color=TUM_GRAY, marker="x")

# plot.add_legend()
# plt.tight_layout(pad=0.2)
# plt.show()
# plt.savefig(f"translation_{sub_folder}.pdf")

# plot = Plot(xlabel="iteration", ylabel="rotation")
# # plt.yscale('log')
# plt.xscale('symlog')
# plot.ax.plot(design_data[0]["rotation"], color=TUM_GRAY_2, label=f"unscaled")
# plot.ax.plot(max_iterations[0], design_data[0]["rotation"][-1:], color=TUM_GRAY_2, marker="x")
# plot.ax.plot(design_data[1]["rotation"], color=TUM_ORANGE, label=f"scaled full diagonal", linestyle=":")
# plot.ax.plot(max_iterations[1], design_data[1]["rotation"][-1:], color=TUM_ORANGE, marker="x")
# plot.ax.plot(design_data[2]["rotation"], color=TUM_BLUE, label=f"scaled mixed")
# plot.ax.plot(max_iterations[2], design_data[2]["rotation"][-1:], color=TUM_BLUE, marker="x")
# plot.ax.plot(design_data[3]["rotation"], color=TUM_GRAY, label=f"scaled w off-diagonal", linestyle=":")
# plot.ax.plot(max_iterations[3], design_data[3]["rotation"][-1:], color=TUM_GRAY, marker="x")

# plot.add_legend(loc="lower right")
# plt.tight_layout(pad=0.2)
# plt.show()
# plt.savefig(f"rotation_{sub_folder}.pdf")
