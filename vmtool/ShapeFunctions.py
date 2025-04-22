#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Shape Functions
#####################################################################

# external imports
import numpy as np

def LinearHatFunction(zeta, zeta_i, n):

    if (zeta_i-n) <= zeta and zeta <= zeta_i:
        N = 1/n + (zeta-zeta_i) / (n**2)

    elif zeta_i <= zeta and zeta <= (zeta_i+n):
        N = 1/n - (zeta-zeta_i) / (n**2)

    else:
        N = 0

    return N

def LinearFilter(zeta, zeta_i, n):

    distance = abs(zeta-zeta_i)

    if distance < n:
        N = 1 - distance / n
    else:
        N = 0

    return N

def LinearNodeShapeFunction(zeta, zeta_i, r_left, r_right):

    if (zeta_i-r_left) <= zeta and zeta <= zeta_i and r_left != 0:
        N = 1 + (zeta-zeta_i) / r_left

    elif zeta_i <= zeta and zeta <= (zeta_i+r_right) and r_right != 0:
        N = 1 - (zeta-zeta_i) / r_right

    else:
        N = 0

    return N

def VaryingLinearFunction(zeta, zeta_i, r, DomainEdges):

    left_edge = DomainEdges[0]
    right_edge = DomainEdges[1]

    distance_to_edges = min(zeta_i - left_edge, right_edge - zeta_i)

    if zeta_i == zeta:
        N = 1

    elif distance_to_edges < r:
        reduced_radius = distance_to_edges
        N = LinearFilter(zeta, zeta_i, reduced_radius)

    else:
        N = LinearFilter(zeta, zeta_i, r)

    return N

def VaryingLinearFunctionv2(zeta, zeta_i, r, DomainEdges):

    left_edge = DomainEdges[0]
    right_edge = DomainEdges[1]

    if zeta_i == zeta:
        N = 1

    elif zeta_i == DomainEdges[0]:
        N = 0

    elif zeta_i == DomainEdges[1]:
        N = 0

    elif zeta_i - left_edge < r:
        distance_to_left_edge = zeta_i - left_edge

        if zeta_i > zeta:
            reduced_radius = distance_to_left_edge
            N = LinearFilter(zeta, zeta_i, reduced_radius)
        else:
            N = LinearFilter(zeta, zeta_i, r)
            # if 2*distance_to_left_edge < r:
            #     reduced_radius = 2*distance_to_left_edge
            #     N = LinearFilter(zeta, zeta_i, reduced_radius)
            # else:
            #     N = LinearFilter(zeta, zeta_i, r)

    elif right_edge - zeta_i < r:
        distance_to_right_edge = right_edge - zeta_i

        if zeta_i < zeta:
            reduced_radius = distance_to_right_edge
            N = LinearFilter(zeta, zeta_i, reduced_radius)
        else:
            N = LinearFilter(zeta, zeta_i, r)
            # if 2*distance_to_right_edge < r:
            #     reduced_radius = 2*distance_to_right_edge
            #     N = LinearFilter(zeta, zeta_i, reduced_radius)
            # else:
            #     N = LinearFilter(zeta, zeta_i, r)

    else:
        N = LinearFilter(zeta, zeta_i, r)

    return N

'''Funktioniert noch nicht:
    - funktioniert nur für diskretisierte integration???
    - erste/letzte filterfunktion (am domain anfang/ende) muss angepasst werden??? (dirac ähnlich?)'''
def VaryingLinearHatFunction(zeta, zeta_i, n, DomainSpace, DistanceBetweenNodes):

    N = 0
    if (zeta_i - DomainSpace[0]) <= n or (DomainSpace[1]-zeta_i) <= n:

        roh = min((zeta_i-DomainSpace[0])/DistanceBetweenNodes, (DomainSpace[1]-zeta_i)/DistanceBetweenNodes)

        if roh == 0:
            if zeta_i <= zeta and zeta >= DomainSpace[0] and zeta <= DomainSpace[1]:
                N = 2* (1 - (zeta-zeta_i) / DistanceBetweenNodes)
            elif zeta_i >= zeta and zeta >= DomainSpace[0] and zeta <= DomainSpace[1]:
                N = 2* (1 + (zeta-zeta_i) / DistanceBetweenNodes)
            if N <= 0:
                N = 0

        elif (zeta_i-roh) <= zeta and zeta <= zeta_i:
            N = (1/roh + (zeta-zeta_i) / (roh**2))

        elif zeta_i <= zeta and zeta <= (zeta_i+roh):
            N = (1/roh - (zeta-zeta_i) / (roh**2))

    else:
        N = LinearHatFunction(zeta, zeta_i, n)

    return N

# def VaryingLinearHatFunction(zeta, zeta_i, n, DomainSpace, DistanceBetweenNodes):

#     N = 0
#     if (zeta_i - DomainSpace[0]) <= n or (DomainSpace[1]-zeta_i) <= n:

#         roh = min((zeta_i-DomainSpace[0]), (DomainSpace[1]-zeta_i))

#         if roh == 0:    # node at domain edge
#             if zeta_i <= zeta and zeta >= DomainSpace[0] and zeta <= DomainSpace[1]:
#                 N = 1/1e-8 - (zeta-zeta_i) / (1e-8**2)
#             elif zeta_i >= zeta and zeta >= DomainSpace[0] and zeta <= DomainSpace[1]:
#                 N = 1/1e-8 + (zeta-zeta_i) / (1e-8**2)
#             if N <= 0:
#                 N = 0

#         elif (zeta_i-roh) <= zeta and zeta <= zeta_i:
#             N = (1/roh + (zeta-zeta_i) / (roh**2))

#         elif zeta_i <= zeta and zeta <= (zeta_i+roh):
#             N = (1/roh - (zeta-zeta_i) / (roh**2))

#     else:
#         N = LinearHatFunction(zeta, zeta_i, n)

#     return N

' Lineare Funktionen werden nicht verwendet'
def LinearFunction1(zeta, zeta_i, zeta_j):
    ''' Linear function between zeta_i and zeta_j
        with N(zeta_i) = 0 and N(zeta_j) = 1'''

    if zeta >= zeta_i and zeta <= zeta_j and zeta_i != zeta_j:
        N =  (zeta - zeta_i) / (zeta_j - zeta_i)
    else:
        N = 0

    return N

def LinearFunction2(zeta, zeta_i, zeta_j):
    ''' Linear function between zeta_i and zeta_j
        with N(zeta_i) = 1 and N(zeta_j) = 0'''

    if zeta >= zeta_i and zeta <= zeta_j and zeta_i != zeta_j:
        N =  1 - (zeta - zeta_i) / (zeta_j - zeta_i)
    else:
        N = 0

    return N


