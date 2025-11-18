#####################################################################
# Technische Universitaet Muenchen
# Lehrstuhl für Statik
# Vertex Morphing Tool
# Author: David Schmölz
# david.schmoelz@tum.de
#####################################################################
# Gaussian Quadrature
#
# Functions which returns the Gauss points and their respective weights.
# All values are taken from Abramowitz: Handbook of Mathematical Functions with Formulas, Graphs, and Mathematical Tables
#
# Parameters:
# number_of_gauss_points            :   number of desired Gauss points
#
# Return:
# points                            :   Gauss points
# weights                           :   weights of each Gauss point
#####################################################################

# external imports
import numpy as np

def CalculateGaussPointsAndWeights(number_of_gauss_points, integration_start, integration_end):

    nGP = number_of_gauss_points
    points = [0 for i in range(nGP) ]
    weights = [0 for i in range(nGP) ]

    if nGP == 1:

        points[0] = 0

        weights[0] = 2

    elif nGP == 2:

        points[0] = - 1 / np.sqrt(3)
        points[1] = - points[0]

        weights[0] = 1
        weights[1] = 1

    elif nGP == 3:

        points[0] = - np.sqrt( 3 / 5 )
        points[1] = 0
        points[2] = - points[0]

        weights[0] = 5 / 9
        weights[1] = 8 / 9
        weights[2] = weights[0]

    elif nGP == 4:

        points[0] = - np.sqrt( ( 3 / 7 ) + ( 2 / 7 ) * np.sqrt( 6 / 5 ) )
        points[1] = - np.sqrt( ( 3 / 7 ) - ( 2 / 7 ) * np.sqrt( 6 / 5 ) )
        points[2] = - points[1]
        points[3] = - points[0]

        weights[0] = ( 18 - np.sqrt(30) ) / 36
        weights[1] = ( 18 + np.sqrt(30) ) / 36
        weights[2] = weights[1]
        weights[3] = weights[0]

    elif nGP == 5:

        points[0] = - ( 1 / 3 ) * np.sqrt( 5 + 2 * np.sqrt( 10 / 7 ) )
        points[1] = - ( 1 / 3 ) * np.sqrt( 5 - 2 * np.sqrt( 10 / 7 ) )
        points[2] = 0
        points[3] = - points[1]
        points[4] = - points[0]

        weights[0] = ( 322 - 13 * np.sqrt(70) ) / 900
        weights[1] = ( 322 + 13 * np.sqrt(70) ) / 900
        weights[2] = 128 / 225
        weights[3] = weights[1]
        weights[4] = weights[0]

    elif nGP == 6:

        points[0] = - 0.932469514203152
        points[1] = - 0.661209386466265
        points[2] = - 0.238619186083197
        points[3] = - points[2]
        points[4] = - points[1]
        points[5] = - points[0]

        weights[0] = 0.171324492379170
        weights[1] = 0.360761573048139
        weights[2] = 0.467913934572691
        weights[3] = weights[2]
        weights[4] = weights[1]
        weights[5] = weights[0]

    elif nGP == 7:

        points[0] = - 0.949107912342759
        points[1] = - 0.741531185599394
        points[2] = - 0.405845151377397
        points[3] = - 0
        points[4] = - points[2]
        points[5] = - points[1]
        points[6] = - points[0]

        weights[0] = 0.129484966168870
        weights[1] = 0.279705391489277
        weights[2] = 0.381830050505119
        weights[3] = 0.417959183673469
        weights[4] = weights[2]
        weights[5] = weights[1]
        weights[6] = weights[0]

    points = [(integration_end-integration_start)/2 * x + (integration_start+integration_end)/2 for x in points]

    return points, weights