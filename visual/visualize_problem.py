#!/usr/bin/python

import math
import matplotlib.pyplot as plt

from typing import List, Tuple

from absl import app
from absl import flags
from absl import logging

FLAGS = flags.FLAGS
flags.DEFINE_string('problem_file', '', 'The problem file path.')
flags.DEFINE_boolean(
    'show_plot', True, 'Whether or not the interactive plot window should be '
    'shown.')
flags.DEFINE_string(
    'plot_file', '', 'Plot to an image file, unless this flag is an empty '
    'string.')


class Problem:
    def __init__(self, problem_path: str):
        logging.info(f'Loading from: {problem_path}')
        with open(problem_path, "r") as fin:
            w, h, self.nd, self.t, self.m = [int(x) for x in next(fin).split()]
            self.np = int(next(fin).split()[0])
            self.product_weight = [int(x) for x in next(fin).split()]
            self.nw = int(next(fin).split()[0])
            self.warehouse_loc = []
            self.warehouse_stock = []
            for w in range(self.nw):
                x, y = [int(x) for x in next(fin).split()]
                self.warehouse_loc.append((x, y))
                self.warehouse_stock.append(
                    [int(x) for x in next(fin).split()])
            self.no = int(next(fin).split()[0])
            self.order_loc = []
            self.order_request = []
            for o in range(self.no):
                x, y = [int(x) for x in next(fin).split()]
                self.order_loc.append((x, y))
                # Eat one line - the one containing # of product items
                next(fin)
                curr_order_request = [0 for x in range(self.np)]
                for x in next(fin).split():
                    curr_order_request[int(x)] += 1
                self.order_request.append(curr_order_request)
        logging.info("Loading done.")


class InteractivePlot:

    _problem: Problem = None
    _current_w: int = -1
    _current_o: int = -1

    @staticmethod
    def run(problem: Problem) -> None:
        """ Call me first, call only me and call me once! """
        InteractivePlot._problem = problem

        plt.close('all')
        fig = plt.figure()
        fig.add_subplot(121)
        cid = fig.canvas.mpl_connect('button_press_event',
                                     InteractivePlot._onclick)
        fig.add_subplot(122)
        plt.gca().invert_yaxis()
        plt.axis('off')

        InteractivePlot._update_plot()

        plt.show()

    @staticmethod
    def _select(loc: Tuple[int, int]) -> None:
        problem = InteractivePlot._problem

        wdist = math.inf
        wbest = -1
        for w in range(problem.nw):
            dx = loc[0] - problem.warehouse_loc[w][0]
            dy = loc[1] - problem.warehouse_loc[w][1]
            dist = dx * dx + dy * dy
            if dist < wdist:
                wdist = dist
                wbest = w

        odist = math.inf
        obest = -1
        for o in range(problem.no):
            dx = loc[0] - problem.order_loc[o][0]
            dy = loc[1] - problem.order_loc[o][1]
            dist = dx * dx + dy * dy
            if dist < odist:
                odist = dist
                obest = o

        if odist < wdist:
            InteractivePlot._current_o = obest
            InteractivePlot._current_w = -1
        else:
            InteractivePlot._current_o = -1
            InteractivePlot._current_w = wbest

    @staticmethod
    def _onclick(event) -> None:
        if event.inaxes != plt.gcf().axes[0]: return
        InteractivePlot._select((event.xdata, event.ydata))
        InteractivePlot._update_plot()

    @staticmethod
    def _print_info(title: str, products: List[int]) -> None:
        fig = plt.gcf()
        plt.sca(fig.axes[1])
        plt.cla()
        plt.title(title)
        plt.gca().invert_yaxis()
        plt.ylim(0, 75)
        plt.xlim(0, 500)
        plt.axis('off')
        i = 0
        for p in range(len(products)):
            if products[p] > 0:
                plt.text((i // 75) * 28,
                         74 - (i % 75),
                         f'{p} : {products[p]}',
                         fontsize=8)
                i += 1
        if i == 0:
            plt.text(0, 74, 'Empty')
        plt.draw()

    @staticmethod
    def _update_plot():
        fig = plt.gcf()
        plt.sca(fig.axes[0])
        plt.cla()
        plt.title("Map")

        problem = InteractivePlot._problem
        current_o = InteractivePlot._current_o
        current_w = InteractivePlot._current_w

        wx = [problem.warehouse_loc[w][0] for w in range(problem.nw)]
        wy = [problem.warehouse_loc[w][1] for w in range(problem.nw)]
        swx = []
        swy = []
        ox = [problem.order_loc[o][0] for o in range(problem.no)]
        oy = [problem.order_loc[o][1] for o in range(problem.no)]
        sox = []
        soy = []
        if current_o >= 0:
            sox.append(problem.order_loc[current_o][0])
            soy.append(problem.order_loc[current_o][1])
        elif current_w >= 0:
            swx.append(problem.warehouse_loc[current_w][0])
            swy.append(problem.warehouse_loc[current_w][1])

        plt.plot(wx, wy, 'ro', swx, swy, 'go', ox, oy, 'b.', sox, soy, 'g.')
        plt.draw()

        if current_w >= 0:
            InteractivePlot._print_info(f'Warehouse {current_w}',
                                        problem.warehouse_stock[current_w])
        elif current_o >= 0:
            InteractivePlot._print_info(f'Order {current_o}',
                                        problem.order_request[current_o])


def plot_to_file(problem: Problem, plot_path: str):
    plt.close('all')
    fig = plt.figure()
    plt.title("Map")
    wx = [problem.warehouse_loc[w][0] for w in range(problem.nw)]
    wy = [problem.warehouse_loc[w][1] for w in range(problem.nw)]
    ox = [problem.order_loc[o][0] for o in range(problem.no)]
    oy = [problem.order_loc[o][1] for o in range(problem.no)]
    plt.plot(wx, wy, 'ro', ox, oy, 'b.')
    plt.savefig(plot_path)


def main(argv):
    problem = Problem(FLAGS.problem_file)
    if FLAGS.plot_file:
        plot_to_file(problem, FLAGS.plot_file)
    if FLAGS.show_plot:
        InteractivePlot.run(problem)


if __name__ == "__main__":
    app.run(main)
