import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
import numpy as np
from mixedvmtool import *
from mixedvmtool.plot_tools.plots import Plot
from mixedvmtool.plot_tools.tum_colors import *

markers = [
    dict(marker='v', fillstyle='none'),
    dict(marker='d', fillstyle='none'),
]

file = "wall_clock_times.csv"

data = pd.read_csv(file, delimiter=",")
data.columns = [x.strip() for x in data.columns]

element_sizes = np.array(data["element_sizes"])
shape_w_off = np.array(data["shape_w_off"])
shape = np.array(data["shape"])

vm_length = (100-20)/2 + 20
node_number = (vm_length / element_sizes + 1)
node_number.astype(int)

plot = Plot(xlabel="number of VM design variables", ylabel='wall-clock time')

plot.ax.plot(node_number, shape,
             color=TUM_BLUE, label="scaled mixed",
             linestyle="-", **markers[1])

plot.ax.plot(node_number, shape_w_off,
             color=TUM_GRAY, label="scaled w off-diagonal",
             linestyle=":", **markers[0])

x_ticks = [61, 1201, 4001, 8001, 12001, 15001]
plot.ax.set_xticks(x_ticks)

# format y tick label, add s to each value for seconds
formatter = FuncFormatter(lambda y, _: f"{y:.0f}s")
plt.gca().yaxis.set_major_formatter(formatter)

plot.fig.legend(loc='upper left', bbox_to_anchor=(0.2, 0.95))
plt.tight_layout(pad=0.2)
plt.savefig("fig_wall_clock_time.pdf")


### Parameterization problem plot
blending_filter = 20
x_limit = 100
element_size = 2
number_of_nodes = int(x_limit/element_size) + 1
vm_x_min_max = [0, int(x_limit/2-blending_filter/2)]
rb_x_min_max = [int(x_limit/2+blending_filter/2), x_limit]

vm = np.arange(vm_x_min_max[0], vm_x_min_max[1]+1, element_size)
transition = np.arange(vm_x_min_max[1], rb_x_min_max[0]+1, element_size)
rb = np.arange(rb_x_min_max[0], rb_x_min_max[1]+1, element_size)
plot = Plot(xlabel=r'$\xi$', ylabel=r'$x$')

plot.ax.plot(transition, np.zeros(transition.size),
             color=TUM_GRAY, label="transition",
             linewidth=0.75,
             marker="|", markersize=4)
plot.ax.plot(vm, np.zeros(vm.size),
             color=TUM_BLUE_5, label="vm",
             linewidth=0.75,
             marker="|", markersize=4)
plot.ax.plot(rb, np.zeros(rb.size),
             color=TUM_ORANGE, label="rb",
             linewidth=0.75,
             marker="|", markersize=4)

plot.ax.set_xticks(np.linspace(0,100,11))
# plot.ax.set_yticks([0])
plot.add_legend(loc="lower left")
plt.tight_layout(pad=0.2)
plt.savefig(f"fig_parameterization.pdf")