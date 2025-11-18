from matplotlib.ticker import TickHelper
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
import os
from plots import *
import logging
from pathlib import Path

logging.basicConfig(filename=f"{Path(__file__).name}.log", level=logging.INFO, filemode="w")


parent_dir = "/home/david/Software/VertexMorphingTool/VertexMorphingTool/Beispiele/"
sub_folder = "nested_max1.0_tol1e-4"
dir = f"{parent_dir}{sub_folder}"

histories = [
    os.path.join(dir, "history_scaling_none"),
    os.path.join(dir, "history_scaling_shape_diag"),
    os.path.join(dir, "history_scaling_shape"),
    os.path.join(dir, "history_scaling_shape_w_off"),
]




# tol = 1e-0
# dfs[0]["mean"] = dfs[0]["df_abs"].rolling(window=3).mean()
# iter0 = dfs[0][dfs[0]["mean"].abs().lt(tol)].index[0]
# iter0 = 50-1
# logging.info(iter0)
# logging.info(dfs[0]["total_objective"][iter0])
# dfs[1]["mean"] = dfs[1]["df_abs"].rolling(window=3).mean()
# iter1 = dfs[1][dfs[1]["mean"].abs().lt(tol)].index[0]
# iter1 = 50-1
# logging.info(iter1)
# logging.info(dfs[1]["total_objective"][iter1])


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
plt.savefig(f"obj_log_{sub_folder}.pdf")


# Translation plot
design_data = []

max_iterations = []
for i, history in enumerate(histories):
    iterations = len(dfs[i]["objective"]) - 1
    max_iterations.append(iterations)
    translation = [0]
    rotation = [0]
    for j in range(iterations):
        path = f"{history}/design_geometry_{j+1}.csv"
        df = pd.read_csv(path, delimiter=",")
        df.columns = [x.strip() for x in df.columns]
        translation.append(float(df["translation"][0]))
        rotation.append(float(df["rotation"][0]))
    data_dict = {
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
plt.savefig(f"translation_{sub_folder}.pdf")

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

plot.add_legend(loc="lower right")
plt.tight_layout(pad=0.2)
plt.show()
plt.savefig(f"rotation_{sub_folder}.pdf")




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
# plot.ax.plot(dfs[0]["objective"], color=TUM_GRAY_2, label=f"unscaled")
# plot.ax.plot(dfs[0]["objective"][-1:], color=TUM_GRAY, marker="x")
# plot.ax.plot(dfs[1]["objective"], color=TUM_ORANGE, label=f"scaled diagonal", linestyle=":")
# plot.ax.plot(dfs[1]["objective"][-1:], color=TUM_ORANGE, marker="x")
# plot.ax.plot(dfs[3]["objective"], color=TUM_GREEN, label=f"scaled w off-diagonal", linestyle=":")
# plot.ax.plot(dfs[3]["objective"][-1:], color=TUM_GREEN, marker="x")
# plot.ax.plot(dfs[2]["objective"], color=TUM_BLUE, label=f"scaled mixed")
# plot.ax.plot(dfs[2]["objective"][-1:], color=TUM_BLUE, marker="x")

# axs_obj_log.plot(dfs[0]["objective"], color=TUM_GRAY_2, label=f"unscaled")
# axs_obj_log.plot(dfs[0]["objective"][-1:], color=TUM_GRAY, marker="x")
# axs_obj_log.plot(dfs[1]["objective"], color=TUM_ORANGE, label=f"scaled diagonal", linestyle=":")
# axs_obj_log.plot(dfs[1]["objective"][-1:], color=TUM_ORANGE, marker="x")
# axs_obj_log.plot(dfs[3]["objective"], color=TUM_GREEN, label=f"scaled w off-diagonal", linestyle=":")
# axs_obj_log.plot(dfs[3]["objective"][-1:], color=TUM_GREEN, marker="x")
# axs_obj_log.plot(dfs[2]["objective"], color=TUM_BLUE, label=f"scaled mixed")
# axs_obj_log.plot(dfs[2]["objective"][-1:], color=TUM_BLUE, marker="x")

# plot.add_legend()
# plt.tight_layout(pad=0.2)
# #plt.show()
# plt.savefig(f"obj_{sub_folder}.pdf")

##### ENDE: Obj Plot mit Zwillingsachse