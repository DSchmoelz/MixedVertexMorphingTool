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


parent_dir = "/home/dschmoelz/Software/VertexMorphingTool/VertexMorphingTool/Beispiele/Plots/SideBySide"
histories = [
    os.path.join(parent_dir, "history_scaling_none"),
    os.path.join(parent_dir, "history_scaling_shape_diag"),
    os.path.join(parent_dir, "history_scaling_shape"),
    os.path.join(parent_dir, "history_scaling_shape_w_off"),
]

dfs = []

for history in histories:
    path = f"{history}/obj_history.csv"
    df = pd.read_csv(path, delimiter=",")
    df.columns = [x.strip() for x in df.columns]
    dfs.append(df)


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


plot = Plot(xlabel="iteration", ylabel="objective")
plt.yscale('log')
plt.xscale('symlog')
plot.ax.plot(dfs[0]["objective"], color=TUM_GRAY, label=f"unscaled")
plot.ax.plot(dfs[0]["objective"][-1:], color=TUM_GRAY, marker="x")
plot.ax.plot(dfs[1]["objective"], color=TUM_ORANGE, label=f"scaled only diagonal")
plot.ax.plot(dfs[1]["objective"][-1:], color=TUM_ORANGE, marker="x")
plot.ax.plot(dfs[2]["objective"], color=TUM_BLUE, label=f"scaled mixed")
plot.ax.plot(dfs[2]["objective"][-1:], color=TUM_BLUE, marker="x")
plot.ax.plot(dfs[3]["objective"], color=TUM_GREEN, label=f"scaled w off-diagonal")
plot.ax.plot(dfs[3]["objective"][-1:], color=TUM_GREEN, marker="x")

plot.add_legend()
plt.tight_layout(pad=0.2)
#plt.show()
plt.savefig("convergence_log.pdf")
