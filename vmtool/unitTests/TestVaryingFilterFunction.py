# #####################################################################
# # Technische Universitaet Muenchen
# # Lehrstuhl für Statik
# # Vertex Morphing Tool
# # Author: David Schmölz
# # david.schmoelz@tum.de
# #####################################################################
# # Adjusted Hat Function Test
# #####################################################################

# import numpy as np
# # import sys
# # import path_setting
# # sys.path.append(path_setting.path)
# from vmtool import *
# import matplotlib.pyplot as plt

# design_number_of_nodes = (17-1)*1+1
# # design_number_of_nodes = 9
# x_i = np.linspace(-8, 8, design_number_of_nodes)
# p_i = np.array([1, 0.7, 2, -1.2, -2.2, -1, 0.4, 1.35, 1.3, 1.25, 1.2, 1.15, 1.1, 1.05, 1, -1, -1])

# DesignMesh = []
# id = 0
# for i in range(0, design_number_of_nodes):
#     DesignMesh.append(DesignNode(id, x_i[i], 0))
#     id += 1

# DesignSpace = np.array([x_i[0], x_i[-1]])

# # FilterRadii = [1, 2, 4, 8, 16, 32]
# FilterRadii = [4]
# for FilterRadius in FilterRadii:
#     HatFunction = np.zeros([design_number_of_nodes, 2001])
#     x = np.linspace(-16,16,2001)

#     plt.figure()
#     for i in range(len(DesignMesh)):
#         design_node_i = DesignMesh[i]
#         for j in range(len(x)):
#             HatFunction[i,j] = ShapeFunctions.VaryingLinearHatFunction(x[j], design_node_i.x, FilterRadius, DesignSpace, 1)

#         plt.plot(x, HatFunction[i,:], '-', label='filter function i = {}'.format(i))

#     # geometry = p_i @ HatFunction

#     # plt.plot(x, geometry, color='lightgrey', label='geometry with filter radius = {}'.format(FilterRadius))
#     plt.legend()

# plt.show()
