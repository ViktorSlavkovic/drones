#!/usr/bin/python

import math
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as ani

from typing import Dict, List, Tuple

from absl import app
from absl import flags
from absl import logging

FLAGS = flags.FLAGS
flags.DEFINE_string('problem_file', '', 'The problem file path.')
flags.DEFINE_string('solution_file', '', 'The solution file path.')
flags.DEFINE_bool(
    'just_calc', False, 'Do not plot not animate, just simulate '
    'and calculate the score.')
flags.DEFINE_bool('show_anim', True,
                  'Show animation, unless just_calc flag is set.')
flags.DEFINE_string(
    'anim_video_file', '', 'Save animation to a video file, unless show_anim '
    'flag is set or this flag is an empty string.')
flags.DEFINE_integer(
    'anim_video_fps', 100, 'Valid only if the animation is being saved to a '
    'video file.')
flags.DEFINE_integer(
    'anim_frames', -1, 'Number of frames to animate, if any animation is '
    'happening and it is >= 0, otherwise the problem\'s T is used')


class ProblemSolution:
    def __init__(self, problem_path: str, solution_path: str):
        logging.info(f'Loading problem from: {problem_path}')
        with open(problem_path, 'r') as fin:
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
            for o in range(self.no):
                x, y = [int(x) for x in next(fin).split()]
                self.order_loc.append((x, y))
                # Eat one line - the one containing # of product items
                next(fin)
                curr_order_request = [0 for _ in range(self.np)]
                for x in next(fin).split():
                    curr_order_request[int(x)] += 1
                self.order_request.append(curr_order_request)
        logging.info('Loading done.')

        logging.info(f'Loading solution from: {solution_path}')
        with open(solution_path, 'r') as fin:
            self.drone_commands = [[] for _ in range(self.nd)]
            Q = int(next(fin).split()[0])
            logging.info(f'Loading {Q} commands.')
            for _ in range(Q):
                cmd_line = next(fin).split()
                cmd = {'type': cmd_line[1]}
                if cmd_line[1] in ['L', 'U']:
                    cmd['warehouse'] = int(cmd_line[2])
                    cmd['product'] = int(cmd_line[3])
                    cmd['num_items'] = int(cmd_line[4])
                elif cmd_line[1] == 'D':
                    cmd['order'] = int(cmd_line[2])
                    cmd['product'] = int(cmd_line[3])
                    cmd['num_items'] = int(cmd_line[4])
                else:
                    cmd['duration'] = int(cmd_line[2])
                self.drone_commands[int(cmd_line[0])].append(cmd)
        logging.info('Loading done')


class Simulation:
    def __init__(self, problem_solution: ProblemSolution):
        self.problem_solution = problem_solution
        self._drone_schedule = set()
        self._load_schedule = set()
        self._unload_schedule = set()
        self._delivery_schedule = set()
        self.warehouse_empty = [False for _ in range(problem_solution.nw)]
        self.order_done = [False for _ in range(problem_solution.no)]
        self.drone_loc = []
        self.drone_travel = []
        for d in range(problem_solution.nd):
            self._drone_schedule.add((0, d))
            self.drone_loc.append(problem_solution.warehouse_loc[0])
            self.drone_travel.append({'s': -100, 'd': 0})
        self.last_tick = -1
        self.total_points = 0.0

    def _command_duration(self, d: int, cmd):
        ps = self.problem_solution
        if cmd['type'] == 'W': return cmd['duration']
        src = ps.order_loc[
            cmd['order']] if cmd['type'] == 'D' else ps.warehouse_loc[
                cmd['warehouse']]
        dx = src[0] - self.drone_loc[d][0]
        dy = src[1] - self.drone_loc[d][1]
        return 1 + math.ceil(math.sqrt(dx * dx + dy * dy))

    def simulate_step(self, tick):
        if self.last_tick >= tick: return
        self.last_tick = tick

        ps = self.problem_solution

        # (1) Handle drones.
        drone_schedule_add = set()
        drone_schedule_remove = set()
        for event in self._drone_schedule:
            if not event[0] == tick: continue
            drone_schedule_remove.add(event)

            d = event[1]
            if len(ps.drone_commands[d]) == 0: continue
            cmd = ps.drone_commands[d][0]
            ps.drone_commands[d] = ps.drone_commands[d][1:]

            cmd_duration = self._command_duration(d, cmd)
            finish_time = tick + cmd_duration - 1
            if finish_time < ps.t - 1:
                drone_schedule_add.add((finish_time + 1, d))

            if cmd['type'] in ['L', 'U']:
                w = cmd['warehouse']
                p = cmd['product']
                n = cmd['num_items']
                if cmd['type'] == 'L':
                    self._load_schedule.add((finish_time, w, p, n, d))
                else:
                    self._unload_schedule.add((finish_time, w, p, n, d))
                travel_time = cmd_duration - 1
                if travel_time > 0:
                    self.drone_travel[d]['s'] = tick
                    self.drone_travel[d]['d'] = travel_time
                    self.drone_travel[d]['fx'] = self.drone_loc[d][0]
                    self.drone_travel[d]['fy'] = self.drone_loc[d][1]
                    self.drone_travel[d]['tx'] = ps.warehouse_loc[w][0]
                    self.drone_travel[d]['ty'] = ps.warehouse_loc[w][1]
                self.drone_loc[d] = ps.warehouse_loc[w]
            elif cmd['type'] == 'D':
                o = cmd['order']
                p = cmd['product']
                n = cmd['num_items']
                self._delivery_schedule.add((finish_time, o, p, n, d))
                travel_time = cmd_duration - 1
                if travel_time > 0:
                    self.drone_travel[d]['s'] = tick
                    self.drone_travel[d]['d'] = travel_time
                    self.drone_travel[d]['fx'] = self.drone_loc[d][0]
                    self.drone_travel[d]['fy'] = self.drone_loc[d][1]
                    self.drone_travel[d]['tx'] = ps.order_loc[o][0]
                    self.drone_travel[d]['ty'] = ps.order_loc[o][1]
                self.drone_loc[d] = ps.order_loc[o]

        self._drone_schedule.difference_update(drone_schedule_remove)
        self._drone_schedule.update(drone_schedule_add)

        # (2) Handle load.
        load_schedule_remove = set()
        for event in self._load_schedule:
            if not event[0] == tick: continue
            load_schedule_remove.add(event)
            w = event[1]
            p = event[2]
            n = event[3]
            ps.warehouse_stock[w][p] -= n
            self.warehouse_empty[w] = (sum(ps.warehouse_stock[w]) == 0)

        self._load_schedule.difference_update(load_schedule_remove)

        # (3) Handle unload.
        unload_schedule_remove = set()
        for event in self._unload_schedule:
            if not event[0] == tick: continue
            unload_schedule_remove.add(event)
            w = event[1]
            p = event[2]
            n = event[3]
            ps.warehouse_stock[w][p] += n
            self.warehouse_empty[w] = (sum(ps.warehouse_stock[w]) == 0)

        self._unload_schedule.difference_update(unload_schedule_remove)

        # (4) Handle delivery.
        delivery_schedule_remove = set()
        for event in self._delivery_schedule:
            if not event[0] == tick: continue
            delivery_schedule_remove.add(event)
            o = event[1]
            p = event[2]
            n = event[3]
            ps.order_request[o][p] -= n
            self.order_done[o] = (sum(ps.order_request[o]) == 0)
            if self.order_done[o]:
                pts = math.ceil((ps.t - tick) * 100.0 / ps.t)
                logging.info(f'Order {o} completed at: {tick} -> {pts}')
                self.total_points += pts

        self._delivery_schedule.difference_update(delivery_schedule_remove)


def calc_plot_data(sim: Simulation):
    wx = []
    wy = []
    ewx = []
    ewy = []
    for w in range(sim.problem_solution.nw):
        x, y = sim.problem_solution.warehouse_loc[w]
        wx.append(x)
        wy.append(y)
        if sim.warehouse_empty[w]:
            ewx.append(x)
            ewy.append(y)
    ox = []
    oy = []
    dox = []
    doy = []
    for o in range(sim.problem_solution.no):
        x, y = sim.problem_solution.order_loc[o]
        ox.append(x)
        oy.append(y)
        if sim.order_done[o]:
            dox.append(x)
            doy.append(y)

    dx = []
    dy = []
    for d in range(sim.problem_solution.nd):
        ts = sim.drone_travel[d]['s']
        te = sim.drone_travel[d]['s'] + sim.drone_travel[d]['d'] - 1
        if ts <= sim.last_tick <= te:
            dt = sim.last_tick - ts
            delta_x = sim.drone_travel[d]['tx'] - sim.drone_travel[d]['fx']
            delta_y = sim.drone_travel[d]['ty'] - sim.drone_travel[d]['fy']
            plot_x = sim.drone_travel[d][
                'fx'] + dt * delta_x / sim.drone_travel[d]['d']
            plot_y = sim.drone_travel[d][
                'fy'] + dt * delta_y / sim.drone_travel[d]['d']
            dx.append(plot_x)
            dy.append(plot_y)
        else:
            dx.append(sim.drone_loc[d][0])
            dy.append(sim.drone_loc[d][1])

    return wx, wy, ewx, ewy, ox, oy, dox, doy, dx, dy


def main(argv):
    ps = ProblemSolution(FLAGS.problem_file, FLAGS.solution_file)
    sim = Simulation(ps)

    if FLAGS.just_calc:
        for tick in range(ps.t):
            sim.simulate_step(tick)
    elif FLAGS.show_anim or FLAGS.anim_video_file:
        fig = plt.figure()

        wx, wy, ewx, ewy, ox, oy, dox, doy, dx, dy = calc_plot_data(sim)
        plot_lines = plt.plot(wx, wy, 'rs', ewx, ewy, 'ks', ox, oy, 'bo', dox,
                              doy, 'ko', dx, dy, 'm1')

        def animate(frame):
            sim.simulate_step(frame)
            _, _, ewx, ewy, _, _, dox, doy, dx, dy = calc_plot_data(sim)
            plot_lines[1].set_data(ewx, ewy)
            plot_lines[3].set_data(dox, doy)
            plot_lines[4].set_data(dx, dy)
            return plot_lines

        num_frames = ps.t if FLAGS.anim_frames < 0 else FLAGS.anim_frames
        anim = ani.FuncAnimation(fig,
                                 animate,
                                 frames=num_frames,
                                 interval=10,
                                 repeat=False)

        if FLAGS.show_anim:
            plt.show()
        else:
            anim.save(FLAGS.anim_video_file,
                      fps=FLAGS.anim_video_fps,
                      extra_args=['-vcodec', 'libx264'])

    if sim.last_tick >= 0:
        print(f'TOTAL SCORE: {int(sim.total_points)}')


if __name__ == "__main__":
    app.run(main)
