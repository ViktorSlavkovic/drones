#!/usr/bin/python

import matplotlib.pyplot as plt
import numpy as np
import random

T = 1000
T_range = range(0, 2*T + 1)

original=np.zeros(len(T_range))
my_lin=np.zeros(len(T_range))
my_exp=np.zeros(len(T_range))
my_heaviside=np.zeros(len(T_range))

# original scoring
for complete_time in range(1, len(T_range)):
  original_value = (T-complete_time) / T
  original[complete_time] = original_value

# my scoring for linear completeness
for complete_time in range(1, len(T_range)):
  completeness=np.zeros(len(T_range))
  for i in range(1, len(T_range)):
    completeness[i] = min(1, i/complete_time)

  my_value = 0.0
  for i in range(1, len(T_range)):
    my_value += completeness[i] * (T-complete_time) / T
  my_lin[complete_time] = my_value / (2*T)

# my scoring for exp completeness
for complete_time in range(1, len(T_range)):
  B = -2.0 * np.log2(10)
  A = -B / complete_time
  completeness=np.zeros(len(T_range))
  for i in range(1, len(T_range)):
    completeness[i] = min(1.0, 2.0**(A*i+B))

  my_value = 0.0
  for i in range(1, len(T_range)):
    my_value += completeness[i] * (T-complete_time) / T
  my_exp[complete_time] = my_value / (2*T)

# my scoring for heaviside completeness
for complete_time in range(1, len(T_range)):
  completeness=np.zeros(len(T_range))
  for i in range(complete_time, len(T_range)):
    completeness[i] = 1

  my_value = 0.0
  for i in range(1, len(T_range)):
    my_value += completeness[i] * (T-complete_time) / T
  my_heaviside[complete_time] = my_value / (2*T)

T_range = np.array(T_range)
T_range = T_range / T

plt.plot(T_range, original, T_range, my_lin, T_range, my_exp, T_range, my_heaviside)
plt.gca().legend(('original_scoring','my_scoring[linear]','my_scoring[exp]','my_scoring[heaviside]'))
plt.xlim(left=T_range[1])
plt.xlabel('order completeness time [T]')
plt.ylabel('score')
plt.grid()
plt.show()
