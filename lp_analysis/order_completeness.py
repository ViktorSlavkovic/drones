#!/usr/bin/python

import math
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
flags.DEFINE_integer('T', 50, 'Simulation time.')
flags.DEFINE_integer('tc', 70, 'Order completion time.')
flags.DEFINE_string(
    'plot_file', 'out.svg',
    'Save plot to the specified file unless this is an '
    'empty string.')


def eval_plot(T, tc, plot_path):
    t = range(0, 2 * T + 1)
    l = []
    e = []
    h = []
    s = []
    for tt in t:
        B = -2.0 * 3.321928094887362
        A = -B / tc
        l.append(min(1.0, tt / tc))
        e.append(min(1.0, 2.0**(A * tt + B)))
        h.append(int(tt >= tc))
        s.append(math.sin(tt / tc * (math.pi / 2)) if tt <= tc else 1)

    fig = plt.figure()
    plt.plot(t, l, t, e, t, h, t, s)
    plt.gca().legend(('lin', 'exp', 'heaviside', 'sin'))
    plt.xlabel('Time')
    plt.ylabel('ORDER_COMPLETENESS')
    plt.grid()
    plt.savefig(plot_path)


def main(argv):
    if FLAGS.plot_file:
        logging.info(f'Saving plot to: {FLAGS.plot_file}')
        eval_plot(FLAGS.T, FLAGS.tc, FLAGS.plot_file)


if __name__ == "__main__":
    app.run(main)
