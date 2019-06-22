#!/usr/bin/python

import subprocess
import sys

import scipy
import psopy
import numpy as np
import matplotlib.pyplot as plt

from absl import app
from absl import flags
from absl import logging

FLAGS = flags.FLAGS
flags.DEFINE_string('opt_method', 'nelder_mead', 'Optimization method to use.')
flags.DEFINE_string(
    'plot_file', '', 'Save plot to the specified file unless this is an empty '
    'string.')
flags.DEFINE_string('eval_bin_path', 'lp_analysis/eval_main',
                    'Path to the cost-fun evaluation binary.')


def eval(x):
    scs = [str(xx) for xx in x]
    cs = ','.join(scs)
    args = [f'--constants={cs}']
    args = [FLAGS.eval_bin_path] + args
    proc = subprocess.Popen(args, stdout=subprocess.PIPE)
    stdout = proc.communicate()[0]
    res = float(stdout)
    return res


def eval_plot(x, plot_path):
    scs = [str(xx) for xx in x]
    cs = ','.join(scs)
    args = ['--verbose', f'--constants={cs}']
    args = [FLAGS.eval_bin_path] + args
    proc = subprocess.Popen(args, stdout=subprocess.PIPE)
    stdout = proc.communicate()[0].decode('ascii')
    stdout_floats = [str(i) for i in stdout.split()]
    sim_t = (len(stdout_floats) - 1) // 10
    t_range = np.arange(0, 2 * sim_t + 1) / sim_t
    original = np.zeros(1 + 2 * sim_t)
    lin = np.zeros(1 + 2 * sim_t)
    exp = np.zeros(1 + 2 * sim_t)
    heaviside = np.zeros(1 + 2 * sim_t)
    sin = np.zeros(1 + 2 * sim_t)
    xx = 0
    for idx in range(len(original)):
        original[idx] = stdout_floats[xx]
        xx += 1
        lin[idx] = stdout_floats[xx]
        xx += 1
        exp[idx] = stdout_floats[xx]
        xx += 1
        heaviside[idx] = stdout_floats[xx]
        xx += 1
        sin[idx] = stdout_floats[xx]
        xx += 1

    fig = plt.figure()
    plt.plot(t_range, original, t_range, lin, t_range, exp, t_range, heaviside,
             t_range, sin)
    plt.gca().legend(('original', 'lin', 'exp', 'heaviside', 'sin'))
    plt.xlabel('Order completion time [T]')
    plt.ylabel('ORDER_SCORE')
    plt.grid()
    plt.savefig(plot_path)


def main(argv):
    opt_values = []

    if FLAGS.opt_method == 'nelder_mead':
        logging.info('Running Nelder-Mead simplex.')
        res = scipy.optimize.minimize(eval,
                                      np.zeros(50 + 1),
                                      method='Nelder-Mead')
        logging.info(f'Got:\n{res}\n')
        opt_values = res.x
    elif FLAGS.opt_method == 'dual_annealing':
        logging.info('Running dual annealing.')
        bounds = [(-10, 10) for _ in range(50 + 1)]
        res = scipy.optimize.dual_annealing(eval, bounds, maxiter=2000)
        logging.info(f'Got:\n{res}\n')
        opt_values = res.x
    elif FLAGS.opt_method == 'particle_swarm':
        logging.info('Running PSO.')
        res = psopy.minimize(eval,
                             np.random.uniform(-10, 10, (100, 50 + 1)),
                             options={'stable_iter': 100})
        logging.info(f'Got:\n{res}\n')
        opt_values = res.x
    else:
        logging.error(f'Unknown optimization method: {FLAGS.opt_method}')
        sys.exit(-1)

    if FLAGS.plot_file:
        logging.info(f'Saving plot to: {FLAGS.plot_file}')
        eval_plot(opt_values, FLAGS.plot_file)


if __name__ == "__main__":
    app.run(main)
