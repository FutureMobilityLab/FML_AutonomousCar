"""File: path_smoothing.py

Description: This file implements a variant of path smoothing based on Bezier curves.
"""

import math
import numpy as np 


class BezierPathSmoothing:

    def __init__(self, num_waypoints=500):
        self._num_waypts = num_waypoints
        self._t = np.linspace(0,1 , num_waypoints)

    #######################
    #   Private Methods
    #######################

    def _binomial(self, n, i):
        result = math.factorial(n) / (math.factorial(i) * math.factorial(n-i))
        return result

    def _basis_func(self, n , i, t):
        result = self._binomial(n, i) * (t**i) * (1 - t) **(n-i)
        return result

    #######################
    #   Public Methods
    #######################

    def compute_smooth_path(self, ctr_points):

        n = len(ctr_points) - 1
        x_bez = np.zeros((1, self._num_waypts))
        y_bez = np.zeros((1, self._num_waypts))

        for i, val in enumerate(ctr_points):
            x, y = val[0], val[1]

            x_bez += self._basis_func(n, i, self._t) * x
            y_bez += self._basis_func(n, i, self._t) * y 

        x_smooth = x_bez.tolist()[0]
        y_smooth = y_bez.tolist()[0]

        return x_smooth, y_smooth

if __name__ == "__main__":
    ctr_points = [(0,0), (20,5), (18, 60)]
    x_data, y_data = BezierPathSmoothing().compute_smooth_path(ctr_points)