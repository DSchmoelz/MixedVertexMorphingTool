from matplotlib.ticker import TickHelper
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
import os
from plots import *
import logging
from pathlib import Path
from TestSideBySideVMRB import CreateTargetMesh, CreateDesignMesh

logging.basicConfig(filename=f"{Path(__file__).name}.log", level=logging.INFO, filemode="w")

parent_dir = "/home/david/Software/VertexMorphingTool/VertexMorphingTool/Beispiele/"
sub_folder = "side_by_side"
dir = f"{parent_dir}{sub_folder}"

histories = [
    os.path.join(dir, "history_scaling_none"),
    os.path.join(dir, "history_scaling_shape_diag"),
    os.path.join(dir, "history_scaling_shape"),
    os.path.join(dir, "history_scaling_shape_w_off"),
]

# Objective plot
dfs = []

for history in histories:
    path = f"{history}/obj_history.csv"
    df = pd.read_csv(path, delimiter=",")
    df.columns = [x.strip() for x in df.columns]
    dfs.append(df)

plot = Plot(xlabel="iteration", ylabel="objective")
# plt.yscale('log')
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
plt.show()
plt.savefig(f"figx_{sub_folder}_obj.pdf")


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
plt.show()
plt.savefig(f"figx_{sub_folder}_translation.pdf")

plot = Plot(xlabel="iteration", ylabel="rotation")
# plt.yscale('log')
plt.xscale('symlog')
plot.ax.plot(design_data[0]["rotation"], color=TUM_GRAY_2, label=f"unscaled")
plot.ax.plot(max_iterations[0], design_data[0]["rotation"][-1:], color=TUM_GRAY_2, marker="x")
plot.ax.plot(design_data[1]["rotation"], color=TUM_ORANGE, label=f"scaled diagonal", linestyle=":")
plot.ax.plot(max_iterations[1], design_data[1]["rotation"][-1:], color=TUM_ORANGE, marker="x")
plot.ax.plot(design_data[2]["rotation"], color=TUM_BLUE, label=f"scaled mixed")
plot.ax.plot(max_iterations[2], design_data[2]["rotation"][-1:], color=TUM_BLUE, marker="x")
plot.ax.plot(design_data[3]["rotation"], color=TUM_GRAY, label=f"scaled w off-diagonal", linestyle=":")
plot.ax.plot(max_iterations[3], design_data[3]["rotation"][-1:], color=TUM_GRAY, marker="x")

plot.add_legend()
plt.tight_layout(pad=0.2)
plt.show()
plt.savefig(f"figx_{sub_folder}_rotation.pdf")

### Optimization problem plot
TargetMesh = CreateTargetMesh(28, 4)
vm_x_min_max = [0, int(4+(28-4)/2-4/2)]
rb_x_min_max = [int(4+(28-4)/2+4/2), 28]
vm = np.arange(vm_x_min_max[0], vm_x_min_max[1]+1)
transition = np.arange(vm_x_min_max[1], rb_x_min_max[0]+1)
rb = np.arange(rb_x_min_max[0], rb_x_min_max[1]+1)
InitialDesignMesh = CreateDesignMesh(28)
plot = Plot()
# plot.ax.plot(InitialDesignMesh.GetNodeCoordinatesX(), InitialDesignMesh.GetShapeZ(), color=TUM_GRAY, label="initial", linewidth=0.75, marker="|", markersize=4)
plot.ax.plot(transition, np.zeros(transition.size),
             color=TUM_GRAY, label="initial transition",
             linewidth=0.75,
             marker="|", markersize=4)
plot.ax.plot(vm, np.zeros(vm.size),
             color=TUM_BLUE_5, label="initial vm",
             linewidth=0.75,
             marker="|", markersize=4)
plot.ax.plot(rb, np.zeros(rb.size),
             color=TUM_ORANGE, label="initial rb",
             linewidth=0.75,
             marker="|", markersize=4)

plot.ax.plot(TargetMesh.GetNodeCoordinatesX(), TargetMesh.GetShapeZ(), color=TUM_BLUE, label="target")

plot.ax.set_xticks([0, 14, 18, 23, 28])
plot.add_legend(loc="lower left")
handles, labels = plt.gca().get_legend_handles_labels()
order = [1,2,0,3]
plt.legend([handles[idx] for idx in order],[labels[idx] for idx in order])
plt.tight_layout(pad=0.2)
plt.show()
plt.savefig(f"figx_{sub_folder}_opt_problem.pdf")

### Shape evolution
labels = ["unscaled", "scaled diagonal", "scaled mixed", "scaled w off-diagonal"]
# plot_steps = [350, 3, 2, 1]
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
    plot.add_legend(loc="lower left")
    plt.tight_layout(pad=0.2)
    plt.show()
    plt.savefig(f"figx_{sub_folder}_shape_evolution_{labels[i].replace(" ", "_")}.pdf")


##### BEGIN: Obj Plot mit Zwillingsachse
# plot = Plot(xlabel="iteration", ylabel="objective")

# plot.ax.set_xlim((0, 20))
# plot.ax.set_xticks([0, 10, 20])
# plot.ax.yaxis.set_ticks_position('left')
# plt.setp(plot.ax.get_xticklabels(), visible=True)

# from mpl_toolkits.axes_grid1 import make_axes_locatable
# divider = make_axes_locatable(plot.ax)
# axs_obj_log = divider.append_axes("right", size=1.0, pad=0, sharey=plot.ax)
# axs_obj_log.grid(True, linestyle=":")
# axs_obj_log.set_xscale('log')
# axs_obj_log.set_xlim((20, 10000))
# axs_obj_log.set_xticks([100, 1000, 2000])
# axs_obj_log.spines['left'].set_visible(False)
# axs_obj_log.yaxis.set_ticks_position('right')
# plt.setp(axs_obj_log.get_yticklabels(), visible=False)

# plt.yscale('log')
# # plt.xscale('symlog')
# plot.ax.plot(dfs[0]["objective"], color=TUM_GRAY, label=f"unscaled")
# plot.ax.plot(dfs[0]["objective"][-1:], color=TUM_GRAY, marker="x")
# plot.ax.plot(dfs[1]["objective"], color=TUM_ORANGE, label=f"scaled only diagonal")
# plot.ax.plot(dfs[1]["objective"][-1:], color=TUM_ORANGE, marker="x")
# plot.ax.plot(dfs[3]["objective"], color=TUM_GREEN, label=f"scaled w off-diagonal")
# plot.ax.plot(dfs[3]["objective"][-1:], color=TUM_GREEN, marker="x")
# plot.ax.plot(dfs[2]["objective"], color=TUM_BLUE, label=f"scaled mixed")
# plot.ax.plot(dfs[2]["objective"][-1:], color=TUM_BLUE, marker="x")

# axs_obj_log.plot(dfs[0]["objective"], color=TUM_GRAY, label=f"unscaled")
# axs_obj_log.plot(dfs[0]["objective"][-1:], color=TUM_GRAY, marker="x")
# axs_obj_log.plot(dfs[1]["objective"], color=TUM_ORANGE, label=f"scaled only diagonal")
# axs_obj_log.plot(dfs[1]["objective"][-1:], color=TUM_ORANGE, marker="x")
# axs_obj_log.plot(dfs[3]["objective"], color=TUM_GREEN, label=f"scaled w off-diagonal")
# axs_obj_log.plot(dfs[3]["objective"][-1:], color=TUM_GREEN, marker="x")
# axs_obj_log.plot(dfs[2]["objective"], color=TUM_BLUE, label=f"scaled mixed")
# axs_obj_log.plot(dfs[2]["objective"][-1:], color=TUM_BLUE, marker="x")

# plot.add_legend()
# plt.tight_layout(pad=0.2)
# #plt.show()
# plt.savefig(f"obj_{sub_folder}.pdf")

##### ENDE: Obj Plot mit Zwillingsachse