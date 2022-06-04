# #####################################################################
# # Technische Universitaet Muenchen
# # Lehrstuhl für Statik
# # Vertex Morphing Tool
# # Author: David Schmölz
# # david.schmoelz@tum.de
# #####################################################################
# # Mapping Test
# #####################################################################

# import numpy as np
# import sys
# import path_setting
# sys.path.append(path_setting.path)
# from vmtool import *
# import matplotlib.pyplot as plt

# # x_i = np.array([0, 2, 3, 3.5, 4.5, 5, 8, 11, 12, 12.5, 13, 16, 18, 20])
# # p_i = np.array([1, 0.7, 2, -1.2, -2.2, -1, 0.4, 1.35, 1.3, 1.25, 1.2, 1.15, 1.1, 1.05, 1, -1, -1])
# x_i = np.arange(10)
# p_i = np.arange(10)

# ids = np.arange(len(x_i))
# # np.random.shuffle(ids)

# ControlNodeList = []
# DesignNodeList = []

# for i in range(0, len(x_i)):
#     ControlNodeList.append(ControlNode(ids[i], x_i[i], p_i[i]))
#     DesignNodeList.append(DesignNode(ids[i], x_i[i], 0))

# ControlMesh = Mesh("control")
# ControlMesh.AddNodes(ControlNodeList)

# DesignMesh = Mesh("design")
# DesignMesh.AddNodes(DesignNodeList)

# Mapper = ForwardMapping.GaussianIntegration(DesignMesh, ControlMesh, 2)
# print(Mapper.GetIntegrationIntervals(DesignMesh.Nodes[5]))
# Mapper.Calculate()
# print(Mapper.MappingMatrix)