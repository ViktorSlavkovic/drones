#!/usr/bin/python

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import numpy as np
import random
import scipy.optimize as opt
from scipy.optimize import OptimizeResult
import subprocess
import psopy
# from psopy import minimize

def eval(x):
  args = [str(xx) for xx in x]
  args = ['./run1'] + args
  proc = subprocess.Popen(args, stdout=subprocess.PIPE)
  stdout = proc.communicate()[0]
  res = float(stdout)
  # print("X = %s -> %.2f" % (str(x), res))
  return res

def eval_plot(x):
  args = [str(xx) for xx in x]
  args = ['./run'] + args
  proc = subprocess.Popen(args, stdout=subprocess.PIPE)
  stdout = proc.communicate()[0].decode('ascii')
  stdout_floats = [str(i) for i in stdout.split()]
  print(len(stdout_floats))
  print(stdout_floats)
  T = (len(stdout_floats) - 1) // 8
  T_range = np.arange(1, 2*T+1) / (2.0*T)
  original = np.zeros(2*T)
  my_lin = np.zeros(2*T)
  my_exp = np.zeros(2*T)
  my_hvs = np.zeros(2*T)
  x = 0
  for i in range(len(original)):
    original[i] = stdout_floats[x]
    x+=1
    my_lin[i] = stdout_floats[x]
    x+=1
    my_exp[i] = stdout_floats[x]
    x+=1
    my_hvs[i] = stdout_floats[x]
    x+=1
  plt.plot(T_range, original, T_range, my_lin, T_range, my_exp, T_range, my_hvs)
  plt.gca().legend(('original_scoring','my_scoring[linear]','my_scoring[exp]','my_scoring[heaviside]'))
  plt.xlim(left=T_range[1])
  plt.xlabel('order completeness time [T]')
  plt.ylabel('score')
  plt.grid()
  plt.show()  

# fun = 10.887
# 2T = 50
# nm_x = [-6.19136999e-05, -9.66992027e-06,  2.02283319e-05, -5.88404147e-05,
#   1.09806055e-04,  1.31479194e-06, -3.57327179e-06,  3.15587269e-05,
#  -6.41542099e-06,  2.39239568e-05,  4.65931378e-06,  1.67648614e-05,
#   6.12606025e-06,  1.30840303e-05,  2.15683932e-05, -1.61250354e-05,
#   2.63578515e-05, -5.19094644e-06,  2.78825017e-05,  6.80734154e-06,
#   1.29754256e-05,  1.28376873e-05, -1.77014132e-05,  3.38704173e-06,
#   5.88073205e-05,  1.06592652e-06, -1.28282666e-05,  9.25642558e-06,
#   1.66327501e-05,  1.94721764e-05,  5.15517116e-06,  1.86460543e-06,
#   1.35095019e-05,  6.25811949e-07,  3.58724764e-06,  1.87007480e-05,
#  -2.83998279e-05,  4.98062304e-05, -4.39905924e-06,  1.70102891e-05,
#   8.40397841e-06, -1.56310926e-05,  2.86493597e-05,  7.49943584e-06,
#  -3.38957080e-05,  3.52560787e-05, -1.23898992e-05,  1.18996826e-05,
#   3.82971882e-07, -3.39651944e-05]

# res = opt.minimize(eval, np.zeros(50), method='Nelder-Mead')

bounds = [(-100, 100) for i in range(50)]

# fun: 10.717
# 2T = 50
# de_x = [-84.09181223,  94.72196161,  71.21835215,  62.63541223,
#         84.69383883, -12.34064231,  82.69203102,  52.85873979,
#         47.5040018 , -23.04623659,  60.43080625,  28.04227055,
#         50.0882215 ,  16.01186364,  31.63329396,  18.54537398,
#         90.7468781 ,   3.50635452,  16.26214797,  74.82342791,
#        -18.0154928 ,  19.17869415,  42.89254235,  78.08451572,
#         -4.14530352,  19.19726823,  43.01275434,  51.04807075,
#         58.95866716,  53.91655371, -11.12441203, -58.37832229,
#         55.48988311,  18.84322797, -14.08840616,  64.05284923,
#         30.80989352,  25.80784785,  46.73255964,  19.70751609,
#         25.38599432,  57.27478054,  31.88363301, -49.3121869 ,
#         10.83238427,   1.75143187,   7.59111531,  10.31240441,
#          9.87549053,  -2.63840593]
# res = opt.differential_evolution(eval, bounds)

# fun: 4.289
# 2T = 50
#da_x = [ 10.92799589, 103.54243021,  59.47623561,  -0.29203944,
#         36.93889563,  28.68831773,  24.77048008,  25.31119171,
#         21.22119462,  44.74010256,  38.97791782,  -9.34706017,
#         49.78309048,  45.40338124,  17.73195665,  -2.72580839,
#         29.4153287 ,  16.77602924,  -9.71072874,  14.25418633]

res = opt.dual_annealing(eval, bounds)

# fun: 10.812
# 50 iter
# 2T = 50
# 100 particles
# pso_x = [ 9.66371905e+01, -3.95535262e+01, -7.32198122e+01,  1.66960923e+02,
#         1.11371514e+02,  1.73372885e+01,  4.80619390e+00,  2.20552992e+01,
#         4.79960583e+01,  2.77035981e+01,  1.98504613e+01, -2.11303201e+01,
#         8.85208619e+01, -3.23760597e+00,  3.24641548e+01,  3.54005683e+01,
#         4.11078521e+01,  3.51906087e+01,  2.63334970e+00,  4.38580916e+01,
#        -5.46002335e+01,  4.25628874e+01,  8.40418545e+01,  1.03519450e+00,
#         1.85154651e+01,  6.54876921e+01,  4.79177657e+01,  1.74639032e+01,
#        -2.47247623e+01,  2.60373315e+01,  2.93376601e+00, -1.96489607e+01,
#         8.63136171e+01,  2.04216338e+01,  5.48859231e+01, -3.18285902e+01,
#         1.72540018e+01,  6.62721789e+01, -3.08730296e+01,  4.79091001e+01,
#         9.14432797e-01,  2.86636344e+01,  8.74058213e-02,  3.34642948e+01,
#        -1.59391453e+01,  3.52627967e+01, -2.17585767e+01, -1.18741252e+00,
#         2.64306386e+01,  3.04441720e+00]
# res = psopy.minimize(eval, np.random.uniform(-100, 100, (100, 50)),
#                      options={'stable_iter': 50})

print(res)

# eval_plot(da_x)

