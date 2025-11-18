#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Bletzinger - Example Filter 1D
#####################################################################

import numpy as np
import sys
import path_setting
sys.path.append(path_setting.path)
from vmtool import *
import matplotlib.pyplot as plt
from numpy.random import default_rng


# def control_geometry(x_j):
#     if x_j < 0:
#         p_j = 3/4 * x_j + 3
#     elif x_j > 0:
#         p_j = -3/4 * x_j + 3
#     else:
#         p_j = 3
#     return p_j

def design_geometry(x_j):

    if x_j < 0:
        p_j = 3/4 * x_j + 3
    elif x_j > 0:
        p_j = -3/4 * x_j + 3
    else:
        p_j = 3
    return p_j

def design_geometry(x_j):
    rng = default_rng()
    return rng.random()

filter_radius = 4
x_limit = 10000
number_of_nodes = x_limit * 1 + 1
x_i = np.linspace(-x_limit, x_limit, number_of_nodes)
p_i = np.zeros(number_of_nodes)
z_i = np.zeros(number_of_nodes)

ControlNodeList = []
control_ids = np.arange(number_of_nodes)
DesignNodeList = []
design_ids = np.arange(number_of_nodes)
for i in range(0, number_of_nodes):
    # p_j[i] = control_geometry(x_i[i])
    ControlNodeList.append(ControlNode(control_ids[i], x_i[i], 0))
    z_i[i] = design_geometry(x_i[i])
    DesignNodeList.append(DesignNode(design_ids[i], x_i[i], z_i[i]))

ControlMesh = Mesh("control")
ControlMesh.AddNodes(ControlNodeList)
DesignMesh = Mesh("design")
DesignMesh.AddNodes(DesignNodeList)

# plt.figure()
# plt.plot(x_i, p_j, '-*', color='lightgrey', label='control polygon')

# Method with Riemann sum
settings = {
    "filter_radius": filter_radius,
    "integration": "RiemannSum",
    "scaling": "none"
}
A = VertexMorphingParameterization.VertexMorphing(DesignMesh, ControlMesh, settings)
A.Calculate()

# z_i = A.MapUpdate(p_j)
Q, R = np.linalg.qr(A.MappingMatrix)
print(f"Q: {np.linalg.det(Q)}")
print(f"R: {np.linalg.det(R)}")
print(f"Determinante R: {np.linalg.det(R)}")

# eig_val, eig_vec = np.linalg.eig(A.MappingMatrix)
# print(f"Determinante: {np.linalg.det(A.MappingMatrix)}")
# print(f"Eigenvalues: {eig_val}")
# print(f"Eigenvectors: {eig_vec}")

# p_i = np.linalg.pinv(B.MappingMatrix) @ z_i_B
p_i = np.linalg.solve(R, Q.transpose() @ z_i)
z_i_least = A.MapUpdate(p_i)
eps = np.linalg.norm(z_i - z_i_least, ord=np.inf)
print(f"Maximale Abweichung: {eps}")
plt.plot(x_i, p_i, '-*', color='grey', label='control polygon by least square solution')
plt.plot(x_i, z_i, '-', label='design shape, r = {}'.format(filter_radius))
plt.plot(x_i, z_i_least, '-', label='design shape by least square solution, r = {}'.format(filter_radius))

plt.legend()
# plt.show()
