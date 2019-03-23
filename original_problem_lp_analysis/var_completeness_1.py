#!/usr/bin/python

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import numpy as np
import random
import scipy.optimize as opt
import subprocess

T = 1000
T_range = range(0, 2*T + 1)

original=np.zeros(2*T+1)
my_lin=np.zeros(2*T+1)
my_exp=np.zeros(2*T+1)
my_heaviside=np.zeros(2*T+1)
completeness1=np.zeros(2*T+1)
completeness2=np.zeros(2*T+1)
completeness3=np.zeros(2*T+1)

def calc_score(completeness, c1, c2):
  res = 0.0
  for t in range(1, len(T_range)):
    tt = t/(2*T)
    res += completeness[t]*(c1*tt+c2)
  res /= 2*T
  return res

def calc_scores(completeness1, completeness2, completeness3, c1, c2):
  res1 = 0.0
  res2 = 0.0
  res3 = 0.0
  for t in range(1, len(T_range)):
    tt = t/(2*T)
    res1 += completeness1[t]*(c1*tt+c2)
    res2 += completeness2[t]*(c1*tt+c2)
    res3 += completeness3[t]*(c1*tt+c2)
  return res1/(2*T),res2/(2*T),res3/(2*T)

def eval(T_range, c1=1, c2=1, plot=True):
  # original scoring
  for complete_time in range(1, len(T_range)):
    original_value = (T-complete_time) / T
    original[complete_time] = original_value

  # my scoring for linear completeness
  for complete_time in range(1, len(T_range)):
    completeness1[0]=0
    for i in range(1, len(T_range)):
      completeness1[i] = min(1, i/complete_time)
    my_lin[complete_time] = calc_score(completeness1, c1, c2)

  # my scoring for exp completeness
  for complete_time in range(1, len(T_range)):
    B = -2.0 * np.log2(10)
    A = -B / complete_time
    completeness2[0]=0
    for i in range(1, len(T_range)):
      completeness2[i] = min(1.0, 2.0**(A*i+B))
    my_exp[complete_time] =  calc_score(completeness2, c1, c2)

  # my scoring for heaviside completeness
  for complete_time in range(1, len(T_range)):
    completeness3[0]=0
    for i in range(1, len(T_range)):
      completeness3[i] = 1 if i >= complete_time else 0
    my_heaviside[complete_time] = calc_score(completeness3, c1, c2)

  if plot:
    T_range = np.array(T_range)
    T_range = T_range / T

    plt.plot(T_range, original, T_range, my_lin, T_range, my_exp, T_range, my_heaviside)
    plt.gca().legend(('original_scoring','my_scoring[linear]','my_scoring[exp]','my_scoring[heaviside]'))
    plt.xlim(left=T_range[1])
    plt.xlabel('order completeness time [T]')
    plt.ylabel('score')
    plt.grid()
    plt.show()
  
  end_max = max(my_lin[-1], my_exp[-1], my_heaviside[-1])
  end_min = min(my_lin[-1], my_exp[-1], my_heaviside[-1])
  total_max = max(max(my_lin), max(my_exp), max(my_heaviside))
  total_min = min(min(my_lin), min(my_exp), min(my_heaviside))
  if total_max - total_min < 0.0001: return 0
  return (end_max-end_min)/(total_max - total_min)

def eval_(c1=1, c2=1, plot=True):
  global T_range
  global my_lin
  global my_exp
  global my_heaviside

  total_max = -np.inf
  total_min = np.inf
  for complete_time in range(1, len(T_range)):
    original[complete_time] = (T-complete_time) / T
    completeness1[0]=0
    completeness2[0]=0
    completeness3[0]=0
    for i in range(1, len(T_range)):
      B = -2.0 * np.log2(10)
      A = -B / complete_time
      completeness1[i] = min(1, i/complete_time)
      completeness2[i] = min(1.0, 2.0**(A*i+B))
      completeness3[i] = 1 if i >= complete_time else 0
    my_lin[complete_time], my_exp[complete_time], my_heaviside[complete_time] = calc_scores(completeness1, completeness2, completeness3, c1, c2)
    total_max=max(total_max, my_lin[complete_time], my_exp[complete_time], my_heaviside[complete_time])
    total_min=min(total_min, my_lin[complete_time], my_exp[complete_time], my_heaviside[complete_time])

  my_lin = 2.0 / (total_max-total_min) * (my_lin - total_min) - 1.0
  my_exp = 2.0 / (total_max-total_min) * (my_exp - total_min) - 1.0
  my_heaviside = 2.0 / (total_max-total_min) * (my_heaviside - total_min) - 1.0

  if plot:
    T_range = np.array(T_range)
    T_range = T_range / T

    plt.plot(T_range, original, T_range, my_lin, T_range, my_exp, T_range, my_heaviside)
    plt.gca().legend(('original_scoring','my_scoring[linear]','my_scoring[exp]','my_scoring[heaviside]'))
    plt.xlim(left=T_range[1])
    plt.xlabel('order completeness time [T]')
    plt.ylabel('score')
    plt.grid()
    plt.show()

  ls = 0.0
  for t in range(1, len(T_range)):
    ls += np.abs(my_lin[t] - original[t])**2 + np.abs(original[t] - my_heaviside[t])**2 + np.abs(my_exp[t] - original[t])**2
  return ls

  # end_max = max(my_lin[-1], my_exp[-1], my_heaviside[-1])
  # end_min = min(my_lin[-1], my_exp[-1], my_heaviside[-1])
  # # total_max = max(max(my_lin), max(my_exp), max(my_heaviside))
  # # total_min = min(min(my_lin), min(my_exp), min(my_heaviside))
  # if total_max - total_min < 0.0001: return 0
  # return (end_max-end_min)/(total_max - total_min)

def eval1(x):
  if x[0] == 0:
    x[0] = 0.01
  res = eval_(x[0], x[1], True)
  print("A = %.2f B = %.2f -> %.2f" % (x[0], x[1], res))
  return res

def eval2(x):
  global T_range
  global original
  global my_lin
  global my_exp
  global my_heaviside

  proc = subprocess.Popen(['./run', '%s'%x[0], '%s'%x[1]], stdout=subprocess.PIPE)
  stdout = proc.communicate()[0]
  stdout_floats = [float(x) for x in stdout.split()]
  res = stdout_floats[-1]

  print("A = %.2f B = %.2f -> %.2f" % (x[0], x[1], res))

  # x = 0
  # for i in range(1, 2*T + 1):
  #   original[i] = stdout_floats[x]
  #   x+=1
  #   my_lin[i] = stdout_floats[x]
  #   x+=1
  #   my_exp[i] = stdout_floats[x]
  #   x+=1
  #   my_heaviside[i] = stdout_floats[x]
  #   x+=1

  # T_range = np.array(T_range)
  # T_range = T_range / T

  # plt.plot(T_range, original, T_range, my_lin, T_range, my_exp, T_range, my_heaviside)
  # plt.gca().legend(('original_scoring','my_scoring[linear]','my_scoring[exp]','my_scoring[heaviside]'))
  # plt.xlim(left=T_range[1])
  # plt.xlabel('order completeness time [T]')
  # plt.ylabel('score')
  # plt.grid()
  # plt.show()

  return res


# print(eval1([1,1]))
# print(eval2([1, 1]))

# print(opt.minimize(eval2, [0,0])) #, method='nelder-mead'

@np.vectorize
def npeval(x, y):
  return eval2([x, y])

X = np.arange(-10, 10, 0.5)
Y = np.arange(-10, 10, 0.5)
X, Y = np.meshgrid(X, Y)
Z = npeval(X, Y)

fig = plt.figure()
ax=fig.gca(projection='3d')
surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm)
plt.show()
# print(eval1([-0.00045209,  0.00054754]))