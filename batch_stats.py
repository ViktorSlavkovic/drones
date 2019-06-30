#!/usr/bin/python

import os
import numpy as np

from absl import app
from absl import flags
from absl import logging

FLAGS = flags.FLAGS
flags.DEFINE_string('tcs_dir', '', '...')
flags.DEFINE_integer('limit_ntcs', 10000, '...')
flags.DEFINE_string('single', '', '...')


class Problem:
    def __init__(self, problem_path: str):
        logging.info(f'Loading problem from: {problem_path}')
        with open(problem_path, "r") as fin:
            _, _, self.nd, self.t, self.m = [int(x) for x in next(fin).split()]
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
            order_weights = []
            all_ordered = np.zeros(self.np)
            for o in range(self.no):
                order_weight = 0
                x, y = [int(x) for x in next(fin).split()]
                self.order_loc.append((x, y))
                # Eat one line - the one containing # of product items
                next(fin)
                curr_order_request = [0 for _ in range(self.np)]
                for x in next(fin).split():
                    curr_order_request[int(x)] += 1
                    order_weight += self.product_weight[int(x)]
                    all_ordered[int(x)] += 1
                order_weights.append(order_weight / self.m)
                self.order_request.append(curr_order_request)

            wcovers = []
            wwcovers = []
            for w in range(self.nw):
                can = 0
                canw = 0
                total = 0
                totalw = 0
                for p in range(self.np):
                    can += min(all_ordered[p], self.warehouse_stock[w][p])
                    canw += self.product_weight[p] * min(
                        all_ordered[p], self.warehouse_stock[w][p])
                    total += all_ordered[p]
                    totalw += self.product_weight[p] * all_ordered[p]
                curr_cover = can / total
                wcovers.append(curr_cover)
                curr_wcover = canw / totalw
                wwcovers.append(curr_wcover)

            ms = [x / self.m for x in self.product_weight]

            self.min_pws = min(ms)
            self.min_wcovers = min(wcovers)
            self.min_wwcovers = min(wwcovers)
            self.min_owcaps = min(order_weights)

            self.max_pws = max(ms)
            self.max_wcovers = max(wcovers)
            self.max_wwcovers = max(wwcovers)
            self.max_owcaps = max(order_weights)

            self.mean_pws = np.mean(ms)
            self.mean_wcovers = np.mean(wcovers)
            self.mean_wwcovers = np.mean(wwcovers)
            self.mean_owcaps = np.mean(order_weights)

            self.std_pws = np.std(ms)
            self.std_wcovers = np.std(wcovers)
            self.std_wwcovers = np.std(wwcovers)
            self.std_owcaps = np.std(order_weights)

        logging.info('Loading done.')


def main(argv):

    min_pws = []
    min_wcovers = []
    min_wwcovers = []
    min_owcaps = []

    max_pws = []
    max_wcovers = []
    max_wwcovers = []
    max_owcaps = []

    mean_pws = []
    mean_wcovers = []
    mean_wwcovers = []
    mean_owcaps = []

    std_pws = []
    std_wcovers = []
    std_wwcovers = []
    std_owcaps = []

    if FLAGS.single != '':
        actual_path = FLAGS.single
        logging.info(f'running: {actual_path}')
        prob = Problem(actual_path)

        min_pws.append(prob.min_pws)
        min_wcovers.append(prob.min_wcovers)
        min_wwcovers.append(prob.min_wwcovers)
        min_owcaps.append(prob.min_owcaps)

        max_pws.append(prob.max_pws)
        max_wcovers.append(prob.max_wcovers)
        max_wwcovers.append(prob.max_wwcovers)
        max_owcaps.append(prob.max_owcaps)

        mean_pws.append(prob.mean_pws)
        mean_wcovers.append(prob.mean_wcovers)
        mean_wwcovers.append(prob.mean_wwcovers)
        mean_owcaps.append(prob.mean_owcaps)

        std_pws.append(prob.std_pws)
        std_wcovers.append(prob.std_wcovers)
        std_wwcovers.append(prob.std_wwcovers)
        std_owcaps.append(prob.std_owcaps)
    else:
        info_path = os.path.join(FLAGS.tcs_dir, 'tc_info.txt')
        with open(info_path, 'r') as info:
            num_tc = int(info.readline())
            num_tc = min(num_tc, FLAGS.limit_ntcs)
            logging.info(f'num_tc={num_tc}')
            for tc in range(0, num_tc):
                line = info.readline().split()
                in_file = line[0]
                actual_path = os.path.join(FLAGS.tcs_dir, in_file)
                logging.info(f'running: {actual_path}')
                prob = Problem(actual_path)

                min_pws.append(prob.min_pws)
                min_wcovers.append(prob.min_wcovers)
                min_wwcovers.append(prob.min_wwcovers)
                min_owcaps.append(prob.min_owcaps)

                max_pws.append(prob.max_pws)
                max_wcovers.append(prob.max_wcovers)
                max_wwcovers.append(prob.max_wwcovers)
                max_owcaps.append(prob.max_owcaps)

                mean_pws.append(prob.mean_pws)
                mean_wcovers.append(prob.mean_wcovers)
                mean_wwcovers.append(prob.mean_wwcovers)
                mean_owcaps.append(prob.mean_owcaps)

                std_pws.append(prob.std_pws)
                std_wcovers.append(prob.std_wcovers)
                std_wwcovers.append(prob.std_wwcovers)
                std_owcaps.append(prob.std_owcaps)

    print(
        f'pws: {np.mean(min_pws)} {np.mean(mean_pws)} {np.mean(max_pws)} / {np.mean(std_pws)}'
    )
    print(
        f'wcovers: {np.mean(min_wcovers)} {np.mean(mean_wcovers)} {np.mean(max_wcovers)} / {np.mean(std_wcovers)}'
    )
    print(
        f'wwcovers: {np.mean(min_wwcovers)} {np.mean(mean_wwcovers)} {np.mean(max_wwcovers)} / {np.mean(std_wwcovers)}'
    )
    print(
        f'owcaps: {np.mean(min_owcaps)} {np.mean(mean_owcaps)} {np.mean(max_owcaps)} / {np.mean(std_owcaps)}'
    )


if __name__ == "__main__":
    app.run(main)
