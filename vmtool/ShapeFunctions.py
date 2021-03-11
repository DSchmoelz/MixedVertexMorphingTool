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

def LinearNodeShapeFunction(zeta, zeta_i, r_left, r_right):

    if (zeta_i-r_left) <= zeta and zeta <= zeta_i and r_left != 0:
        N = 1 + (zeta-zeta_i) / r_left

    elif zeta_i <= zeta and zeta <= (zeta_i+r_right) and r_right != 0:
        N = 1 - (zeta-zeta_i) / r_right

    else:
        N = 0

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


