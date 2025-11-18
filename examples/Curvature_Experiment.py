#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Curvature Experiment - Bletzinger - Example Filter 1D
#####################################################################

import numpy as np
import sys
import path_setting
sys.path.append(path_setting.path)
from mixedvmtool import *
import matplotlib
import matplotlib.pyplot as plt

# plt.style.use('seaborn')
SMALL_SIZE = 8
MEDIUM_SIZE = 10
BIGGER_SIZE = 12

plt.rc('font', size=BIGGER_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=BIGGER_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=14)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=MEDIUM_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=MEDIUM_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=BIGGER_SIZE)    # legend fontsize
plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

def control_geometry(x, radius, height):
    if x < 0 and x >= -radius:
        p = height/radius * x + height
    elif x > 0 and x <= radius:
        p = -height/radius * x + height
    elif x == 0:
        p = height
    else:
        p = 0

    return p


def discrete_curvature_osculating_circle(x, z):
# Crane: Lecture 1 (kappa_D)

    kappa = np.zeros(len(x))
    for i in range(len(x)):
        if i == 0 or i == len(x) - 1:
            pass
        else:
            gamma_h = np.array([x[i-1], z[i-1]])
            gamma_i = np.array([x[i], z[i]])
            gamma_j = np.array([x[i+1], z[i+1]])
            w_i = np.linalg.norm(gamma_j - gamma_h)
            edge_hi = gamma_i - gamma_h
            edge_ij = gamma_j - gamma_i
            theta_i = np.arccos(np.dot(edge_hi, edge_ij) / (np.linalg.norm(edge_hi) * np.linalg.norm(edge_ij)))
            kappa[i] = 2 * np.sin(theta_i) / w_i

    return kappa

# filter_radii = np.linspace(0.1, 1, 9, endpoint=False)
# filter_radii = np.append(filter_radii, np.linspace(1, 10, 10))
filter_radii = np.linspace(1, 10, 10)
height = 1
figure, axis = plt.subplots(2, 1, figsize=[12.0,8.0])

kappa_r = np.zeros(len(filter_radii))

color_map_values = np.linspace(0.4, 0.8, num=filter_radii.size)
color_map = matplotlib.cm.get_cmap('Greys')

j = 0
for filter_radius in filter_radii:
    print("Start calculation for radius : r = {}".format(filter_radius))
    x_limit = 2*filter_radius
    if x_limit < 2:
        x_limit = 2
    # x_limit = 2*16
    control_number_of_nodes = int((x_limit)*2+1)
    x_j = np.linspace(-x_limit, x_limit, control_number_of_nodes)
    p_j = np.zeros(control_number_of_nodes)

    ControlNodeList = []
    control_ids = np.arange(control_number_of_nodes)
    for i in range(0, control_number_of_nodes):
        p_j[i] = control_geometry(x_j[i], filter_radius, height)
        ControlNodeList.append(ControlNode(control_ids[i], x_j[i], p_j[i]))

    ControlMesh = Mesh("control")
    ControlMesh.AddNodes(ControlNodeList)

    design_number_of_nodes = control_number_of_nodes
    x_i = np.linspace(-x_limit, x_limit, design_number_of_nodes)

    DesignNodeList = []
    design_ids = np.arange(design_number_of_nodes)
    for i in range(0, design_number_of_nodes):
        DesignNodeList.append(DesignNode(design_ids[i], x_i[i], 0))

    DesignMesh = Mesh("design")
    DesignMesh.AddNodes(DesignNodeList)

    # axis[0].plot(x_j, p_j, '-*', color='lightgrey', label='control polygon')


    # Method B with Riemann sum
    settings_B = {
        "filter_radius": filter_radius,
        "integration": "RiemannSum",
        "scaling": "none"
    }
    B = VertexMorphingParameterization.VertexMorphing(DesignMesh, ControlMesh, settings_B)
    B.Calculate()

    z_i_B = B.MapUpdate(p_j)

    axis[0].plot(x_i, z_i_B, '-', color=color_map(color_map_values[j]), label='r = {}'.format(filter_radius))

    kappa_r[j] = max(discrete_curvature_osculating_circle(x_i, z_i_B))
    print("Curvature calculation finished for radius : r = {}".format(filter_radius))
    j += 1



axis[1].plot(filter_radii, kappa_r)
axis[1].set(xlabel="$r$", ylabel="$\Delta\kappa$")

axis[0].legend(title="design shapes", loc='upper center', bbox_to_anchor=(1.05, 1.05),
               ncol=1, shadow=True)
#axis[0].axis('equal')
axis[0].grid()
axis[1].grid()
plt.legend()
figure.savefig("Plots/Curvature/experiment.png", dpi=600)
# plt.show()
