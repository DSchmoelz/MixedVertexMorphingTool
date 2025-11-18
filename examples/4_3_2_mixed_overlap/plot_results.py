import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import shutil
from mixedvmtool.plot_tools.plots import Plot
from mixedvmtool.plot_tools.tum_colors import *
from run_mixed_overlap import CreateTargetMesh, CreateDesignMesh

result_folder = "results"
figure_folder = "figures"
if os.path.exists(f"{figure_folder}"):
    shutil.rmtree(f"{figure_folder}")
os.makedirs(f"{figure_folder}")

histories = [
    os.path.join(result_folder, "history_scaling_none"),
    os.path.join(result_folder, "history_scaling_shape_diag"),
    os.path.join(result_folder, "history_scaling_shape"),
    os.path.join(result_folder, "history_scaling_shape_w_off"),
]

# Objective plot
dfs = []

for history in histories:
    path = f"{history}/obj_history.csv"
    df = pd.read_csv(path, delimiter=",")
    df.columns = [x.strip() for x in df.columns]
    dfs.append(df)

plot = Plot(xlabel="iteration", ylabel="objective")
plt.xscale('symlog')
plot.ax.plot(dfs[0]["objective"], color=TUM_GRAY_2, label=f"unscaled")
plot.ax.plot(dfs[0]["objective"][-1:], color=TUM_GRAY_2, marker="x")
plot.ax.plot(dfs[1]["objective"], color=TUM_ORANGE, label=f"scaled diagonal", linestyle=":")
plot.ax.plot(dfs[1]["objective"][-1:], color=TUM_ORANGE, marker="x")
plot.ax.plot(dfs[2]["objective"], color=TUM_BLUE, label=f"scaled mixed")
plot.ax.plot(dfs[2]["objective"][-1:], color=TUM_BLUE, marker="x")
plot.ax.plot(dfs[3]["objective"], color=TUM_GRAY, label=f"scaled w off-diagonal", linestyle=":")
plot.ax.plot(dfs[3]["objective"][-1:], color=TUM_GRAY, marker="x")

plot.add_legend()
plt.tight_layout(pad=0.2)
plt.savefig(f"{figure_folder}/fig_obj.pdf")


# Translation plot
design_data = []
max_iterations = []
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

plot = Plot(xlabel="iteration", ylabel="translation")
# plt.yscale('log')
plt.xscale('symlog')
plot.ax.plot(design_data[0]["translation"], color=TUM_GRAY_2, label=f"unscaled")
plot.ax.plot(max_iterations[0], design_data[0]["translation"][-1:], color=TUM_GRAY_2, marker="x")
plot.ax.plot(design_data[1]["translation"], color=TUM_ORANGE, label=f"scaled diagonal", linestyle=":")
plot.ax.plot(max_iterations[1], design_data[1]["translation"][-1:], color=TUM_ORANGE, marker="x")
plot.ax.plot(design_data[2]["translation"], color=TUM_BLUE, label=f"scaled mixed")
plot.ax.plot(max_iterations[2], design_data[2]["translation"][-1:], color=TUM_BLUE, marker="x")
plot.ax.plot(design_data[3]["translation"], color=TUM_GRAY, label=f"scaled w off-diagonal", linestyle=":")
plot.ax.plot(max_iterations[3], design_data[3]["translation"][-1:], color=TUM_GRAY, marker="x")

plot.add_legend()
plt.tight_layout(pad=0.2)
plt.savefig(f"{figure_folder}/fig_translation.pdf")

plot = Plot(xlabel="iteration", ylabel="rotation")
plt.xscale('symlog')
plot.ax.plot(design_data[0]["rotation"], color=TUM_GRAY_2, label=f"unscaled")
plot.ax.plot(max_iterations[0], design_data[0]["rotation"][-1:], color=TUM_GRAY_2, marker="x")
plot.ax.plot(design_data[1]["rotation"], color=TUM_ORANGE, label=f"scaled diagonal", linestyle=":")
plot.ax.plot(max_iterations[1], design_data[1]["rotation"][-1:], color=TUM_ORANGE, marker="x")
plot.ax.plot(design_data[2]["rotation"], color=TUM_BLUE, label=f"scaled mixed")
plot.ax.plot(max_iterations[2], design_data[2]["rotation"][-1:], color=TUM_BLUE, marker="x")
plot.ax.plot(design_data[3]["rotation"], color=TUM_GRAY, label=f"scaled w off-diagonal", linestyle=":")
plot.ax.plot(max_iterations[3], design_data[3]["rotation"][-1:], color=TUM_GRAY, marker="x")

plot.add_legend(loc="lower right")
plt.tight_layout(pad=0.2)
plt.savefig(f"{figure_folder}/fig_rotation.pdf")

### Optimization problem plot
TargetMesh = CreateTargetMesh(12, 4)
InitialDesignMesh = CreateDesignMesh(12)

rb_x_min_max = [-12, 12]
vm_x_min_max = [-4, 4]
transition_1 = np.arange(vm_x_min_max[0]-2, vm_x_min_max[0]+0.5, 0.5)
transition_2 = np.arange(vm_x_min_max[1], vm_x_min_max[1]+2+0.5, 0.5)
vm = np.arange(vm_x_min_max[0], vm_x_min_max[1]+0.5, 0.5)
rb_1 = np.arange(rb_x_min_max[0], transition_1[0]+0.5, 0.5)
rb_2 = np.arange(transition_2[-1], rb_x_min_max[1]+0.5, 0.5)
plot = Plot()
plot.ax.plot(transition_1, np.zeros(transition_1.size),
             color=TUM_GRAY, label="initial transition",
             linewidth=0.75,
             marker="|", markersize=4)
plot.ax.plot(transition_2, np.zeros(transition_2.size),
             color=TUM_GRAY,
             linewidth=0.75,
             marker="|", markersize=4)
plot.ax.plot(rb_1, np.zeros(rb_1.size),
             color=TUM_ORANGE, label="initial rb",
             linewidth=0.75,
             marker="|", markersize=4)
plot.ax.plot(rb_2, np.zeros(rb_2.size),
             color=TUM_ORANGE,
             linewidth=0.75,
             marker="|", markersize=4)
plot.ax.plot(vm, np.zeros(vm.size),
             color=TUM_BLUE_5, label="initial vm+rb",
             linewidth=0.75,
             marker="|", markersize=4)

plot.ax.plot(TargetMesh.GetNodeCoordinatesX(), TargetMesh.GetShapeZ(), color=TUM_BLUE, label="target")
plot.ax.set_xticks([-12, -6, -4, 0, 4, 6, 12])
plot.add_legend(loc="upper left")
handles, labels = plt.gca().get_legend_handles_labels()
order = [1,2,0,3]
plt.legend([handles[idx] for idx in order],[labels[idx] for idx in order])
plt.tight_layout(pad=0.2)
plt.savefig(f"{figure_folder}/fig_opt_problem.pdf")

### Shape evolution
labels = ["unscaled", "scaled diagonal", "scaled mixed", "scaled w off-diagonal"]
number_of_plots = 5
for i in range(len(design_data)):
    plot = Plot()
    plot.ax.plot(InitialDesignMesh.GetNodeCoordinatesX(), InitialDesignMesh.GetShapeZ(), color=TUM_BLUE_5, label="initial", linewidth=0.75, marker="|", markersize=4)
    plot.ax.plot(TargetMesh.GetNodeCoordinatesX(), TargetMesh.GetShapeZ(), color=TUM_BLUE, label="target")
    iterations = len(dfs[i]["objective"]) - 1
    # number_of_plots = int(iterations/plot_steps[i])
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
    plot.add_legend(loc="upper left")
    plt.tight_layout(pad=0.2)
    plt.savefig(f"{figure_folder}/fig_shape_evolution_{labels[i].replace(" ", "_")}.pdf")
