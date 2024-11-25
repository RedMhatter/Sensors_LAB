import numpy as np
from numpy.linalg import inv
import math
from math import cos, sin, sqrt
import numpy as np
import sympy
from sympy import symbols, Matrix

#decorator
def squeeze_sympy_out(func):
    # inner function
    def squeeze_out(*args):
        out = func(*args).squeeze()
        return out
    return squeeze_out

def eval_hx_odom(v, w, sigma):
    """""
    Sampling z from landmark model for range and bearing
    """""
    v_ = v + np.random.normal(0., sigma[0])
    w_ = w + np.random.normal(0., sigma[1])
    return np.array([v_, w_])

def Ht_odom():
    v, w = symbols("v, w")

    hx = Matrix(
        [
            [v],
            [w],
        ]
    )
    eval_hx = squeeze_sympy_out(sympy.lambdify((v, w), hx, "numpy"))
    
    Ht = hx.jacobian(Matrix([v, w]))
    eval_Ht = squeeze_sympy_out(sympy.lambdify((v, w), Ht, "numpy"))

    print("Ht:", Ht)

    return eval_hx, eval_Ht




def eval_hx_imu(w, sigma):
    """""
    Sampling z from landmark model for range and bearing
    """""
    w_ = w + np.random.normal(0., sigma[0])
    return np.array([w_])

def Ht_imu():
    w = symbols("w")

    hx = Matrix(
        [
            [w],
        ]
    )
    eval_hx = squeeze_sympy_out(sympy.lambdify((w), hx, "numpy"))
    
    Ht = hx.jacobian(Matrix([w]))
    eval_Ht = squeeze_sympy_out(sympy.lambdify((w), Ht, "numpy"))

    print("Ht:", Ht)

    return eval_hx, eval_Ht