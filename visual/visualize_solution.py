#!/usr/bin/python

import math
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as ani
import sys

from absl import app
from absl import flags
from absl import logging

FLAGS = flags.FLAGS
flags.DEFINE_string('problem_file', '', 'The problem file path.')
flags.DEFINE_string('solution_file', '', 'The solution file path.')
flags.DEFINE_bool('just_calc', False, 'Do not plot not animate, just simulate '
                                      'and calculate the score.')

W = 0
H = 0
Nd = 0
Np = 0
Nw = 0
No = 0
T = 0
M = 0

warehouse_loc_x = []
warehouse_loc_y = []
warehouse_stock = []
warehouse_empty = []
order_loc_x = []
order_loc_y = []
order_request = []
order_done = []
drone_loc_x = []
drone_loc_y = []
drone_commands = []

plot_lines = []
total_points = 0.0

def load_problem(file_path):
  global W, H, Nd, Np, Nw, No, T, M
  global warehouse_loc_x, warehouse_loc_y, warehouse_stock, warehouse_empty
  global order_loc_x, order_loc_y, order_request, order_done
  global drone_loc_x, drone_loc_y

  logging.info("Loading problem from: %s" % file_path)
  
  with open(file_path, "r") as fin:
    W, H, Nd, T, M = [int(x) for x in next(fin).split()]
    assert 1 <= W <= 100000
    assert 1 <= H <= 100000
    assert 1 <= Nd <= 1000
    assert 1 <= T <= 1000000
    assert 1 <= M <= 10000

    Np = int(next(fin).split()[0])
    assert 1 <= Np <= 10000
    product_weight = [int(x) for x in next(fin).split()]
    for pw in product_weight: assert 1 <= pw <= M

    Nw = int(next(fin).split()[0])
    assert 1 <= Nw <= 10000
    for w in range(Nw):
      x, y = [int(x) for x in next(fin).split()]
      assert 0 <= x < W
      assert 0 <= y < H
      warehouse_loc_x.append(x)
      warehouse_loc_y.append(y)
      stock = [int(x) for x in next(fin).split()] 
      for p in stock: assert 0 <= p <= 10000
      warehouse_stock.append(stock)
      warehouse_empty.append(sum(stock) == 0)

    No = int(next(fin).split()[0])
    assert 1 <= No <= 10000
    for o in range(No):
      x, y = [int(x) for x in next(fin).split()]
      assert 0 <= x < W
      assert 0 <= y < H
      order_loc_x.append(x)
      order_loc_y.append(y)
      next(fin)  # Eat one line - the one containing # of product items
      curr_order_request = [0 for x in range(Np)]
      for x in next(fin).split():
        p = int(x)
        assert 0 <= p < Np
        curr_order_request[p] += 1 
      order_request.append(curr_order_request)
      assert(sum(curr_order_request) > 0)
      order_done.append(False)

  drone_loc_x = [warehouse_loc_x[0] for d in range(Nd)]
  drone_loc_y = [warehouse_loc_y[0] for d in range(Nd)]

  logging.info("Loading done.")

def load_solution(file_path):
  global drone_commands

  logging.info("Loading solution from: %s" % file_path)

  drone_commands = [[] for d in range(Nd)]
  with open(file_path, "r") as fin:
    Q = int(next(fin).split()[0])
    assert 0 <= Q <= Nd * T
    logging.info('Loading %d commands.' % Q)
    for q in range(Q):
      cmd_line = next(fin).split()
      assert 0 <= int(cmd_line[0]) < Nd
      cmd = { 'type' : cmd_line[1] }
      assert cmd['type'] in ['L', 'U', 'D', 'W']
      if cmd_line[1] in ['L', 'U']:
        cmd['warehouse'] = int(cmd_line[2])
        assert(0 <= cmd['warehouse'] < Nw)
        cmd['product'] = int(cmd_line[3])
        assert(0 <= cmd['product'] < Np)
        cmd['num_items'] = int(cmd_line[4])
        assert(1 <= cmd['num_items'])
      elif cmd_line[1] == 'D':
        cmd['order'] = int(cmd_line[2])
        assert(0 <= cmd['order'] < No)
        cmd['product'] = int(cmd_line[3])
        assert(0 <= cmd['product'] < Np)
        cmd['num_items'] = int(cmd_line[4])
        assert(1 <= cmd['num_items'])
      else:
        cmd['duration'] = int(cmd_line[2])
        assert(1 <= cmd['duration'] <= T)

      drone_commands[int(cmd_line[0])].append(cmd)
  
  logging.info("Loading done.")

drone_schedule = set()
unload_schedule = set()
delivery_schedule = set()
drone_plot_loc_x = []
drone_plot_loc_y = []
drone_travel = []

def init_simulation():
  global drone_schedule
  global drone_travel
  for d in range(Nd):
    drone_schedule.add((0, d))
  drone_travel = [{'s': -1, 'd': 0} for d in range(Nd)]

def command_duration(d, cmd):
  if cmd['type'] == 'W': return cmd['duration']
  dx = (order_loc_x[cmd['order']] if cmd['type'] == 'D' else warehouse_loc_x[cmd['warehouse']]) - drone_loc_x[d]
  dy = (order_loc_y[cmd['order']] if cmd['type'] == 'D' else warehouse_loc_y[cmd['warehouse']]) - drone_loc_y[d]
  return 1 + math.ceil(math.sqrt(dx * dx + dy * dy))

last_simulation_step = -1
def simulate_step(tick):
  global last_simulation_step
  if last_simulation_step >= tick: return
  last_simulation_step = tick
  
  # logging.info('simulate_step(%d)' % tick)
  assert tick < T

  global drone_loc_x, drone_loc_y
  global warehouse_empty, warehouse_stock, warehouse_loc_x, warehouse_loc_y
  global order_done, order_request, order_loc_x, order_loc_y
  global total_points
  global drone_schedule, unload_schedule, delivery_schedule
  global drone_travel
  global drone_plot_loc_x, drone_plot_loc_y

  # (1) Handle drones.
  drone_schedule_add = set()
  drone_schedule_remove = set()
  for event in drone_schedule:
    if not event[0] == tick: continue
    drone_schedule_remove.add(event)
    
    d = event[1]
    if len(drone_commands[d]) == 0: continue
    cmd = drone_commands[d][0]
    drone_commands[d] = drone_commands[d][1:]
    
    cmd_duration = command_duration(d, cmd)
    finish_time = tick + cmd_duration - 1
    if finish_time < T - 1: drone_schedule_add.add((finish_time + 1, d))
    
    if cmd['type'] == 'L':
      w = cmd['warehouse']
      p = cmd['product']
      n = cmd['num_items']
      warehouse_stock[w][p] -= n
      warehouse_empty[w] = (sum(warehouse_stock[w]) == 0)
      travel_time = cmd_duration - 1
      if travel_time > 0:
        drone_travel[d]['s'] = tick
        drone_travel[d]['d'] = travel_time
        drone_travel[d]['fx'] = drone_loc_x[d]
        drone_travel[d]['fy'] = drone_loc_y[d]
        drone_travel[d]['tx'] = warehouse_loc_x[w]
        drone_travel[d]['ty'] = warehouse_loc_y[w]
      drone_loc_x[d] = warehouse_loc_x[w]
      drone_loc_y[d] = warehouse_loc_y[w]
    elif cmd['type'] == 'U':
      w = cmd['warehouse']
      p = cmd['product']
      n = cmd['num_items']
      unload_schedule.add((finish_time, w, p, n, d))
      travel_time = cmd_duration - 1
      if travel_time > 0:
        drone_travel[d]['s'] = tick
        drone_travel[d]['d'] = travel_time
        drone_travel[d]['fx'] = drone_loc_x[d]
        drone_travel[d]['fy'] = drone_loc_y[d]
        drone_travel[d]['tx'] = warehouse_loc_x[w]
        drone_travel[d]['ty'] = warehouse_loc_y[w]
      drone_loc_x[d] = warehouse_loc_x[w]
      drone_loc_y[d] = warehouse_loc_y[w]
    elif cmd['type'] == 'D':
      o = cmd['order']
      p = cmd['product']
      n = cmd['num_items']
      delivery_schedule.add((finish_time, o, p, n, d))
      travel_time = cmd_duration - 1
      if travel_time > 0:
        drone_travel[d]['s'] = tick
        drone_travel[d]['d'] = travel_time
        drone_travel[d]['fx'] = drone_loc_x[d]
        drone_travel[d]['fy'] = drone_loc_y[d]
        drone_travel[d]['tx'] = order_loc_x[o]
        drone_travel[d]['ty'] = order_loc_y[o]
      drone_loc_x[d] = order_loc_x[o]
      drone_loc_y[d] = order_loc_y[o]

  drone_schedule.difference_update(drone_schedule_remove)
  drone_schedule.update(drone_schedule_add)
  
  # (2) Handle unload.
  unload_schedule_remove = set()
  for event in unload_schedule:
    if not event[0] == tick: continue
    unload_schedule_remove.add(event)
    w = event[1]
    p = event[2]
    n = event[3]
    warehouse_stock[w][p] += n
    warehouse_empty[w] = (sum(warehouse_stock[w] == 0))

  unload_schedule.difference_update(unload_schedule_remove)

  # (3) Handle delivery.
  delivery_schedule_remove = set()
  for event in delivery_schedule:
    if not event[0] == tick: continue
    delivery_schedule_remove.add(event)
    o = event[1]
    p = event[2]
    n = event[3]
    order_request[o][p] -= n
    order_done[o] = (sum(order_request[o]) == 0)
    if order_done[o]:
      pts = math.ceil((T - tick) * 100.0 / T)
      logging.info('Order %d completed at: %d -> %d' % (o, tick, pts))
      total_points += pts

  delivery_schedule.difference_update(delivery_schedule_remove)

  # (4) Calculate drone plot locations
  drone_plot_loc_x = []
  drone_plot_loc_y = []
  for d in range(Nd):
    if drone_travel[d]['s'] <= tick <= drone_travel[d]['s'] + drone_travel[d]['d'] - 1:
      dt = tick - drone_travel[d]['s']
      dx = drone_travel[d]['tx'] - drone_travel[d]['fx']
      dy = drone_travel[d]['ty'] - drone_travel[d]['fy']
      drone_plot_loc_x.append(drone_travel[d]['fx'] + dt * dx / drone_travel[d]['d'])
      drone_plot_loc_y.append(drone_travel[d]['fy'] + dt * dy / drone_travel[d]['d'])
    else:
      drone_plot_loc_x.append(drone_loc_x[d])
      drone_plot_loc_y.append(drone_loc_y[d])

def animate(frame):
  simulate_step(frame)
  ewx = [warehouse_loc_x[w] for w in range(Nw) if warehouse_empty[w]]
  ewy = [warehouse_loc_y[w] for w in range(Nw) if warehouse_empty[w]]
  dox = [order_loc_x[o] for o in range(No) if order_done[o]]
  doy = [order_loc_y[o] for o in range(No) if order_done[o]]
  global plot_lines
  plot_lines[1].set_data(ewx, ewy)
  plot_lines[3].set_data(dox, doy)
  plot_lines[4].set_data(drone_plot_loc_x, drone_plot_loc_y)
  return plot_lines

def main(argv):
  load_problem(FLAGS.problem_file)
  load_solution(FLAGS.solution_file)
  init_simulation()

  if FLAGS.just_calc:
    for tick in range(T): simulate_step(tick)
  else:
    fig = plt.figure()
  
    ewx = [warehouse_loc_x[w] for w in range(Nw) if warehouse_empty[w]]
    ewy = [warehouse_loc_y[w] for w in range(Nw) if warehouse_empty[w]]
    dox = [order_loc_x[o] for o in range(No) if order_done[o]]
    doy = [order_loc_y[o] for o in range(No) if order_done[o]]
    global plot_lines
    plot_lines = plt.plot(warehouse_loc_x, warehouse_loc_y, 'rs',
                          ewx, ewy, 'ks',
                          order_loc_x, order_loc_y, 'bo',
                          dox, doy, 'ko',
                          drone_loc_x, drone_loc_y, 'm1')

    anim = ani.FuncAnimation(fig, animate, frames=T,
                             interval = 1000 / 100,
                             repeat=False)

    anim.save('anim.mp4', fps=100, extra_args=['-vcodec', 'libx264'])

    plt.show()
  
  logging.info('TOTAL POINTS = %d' % total_points)

if __name__== "__main__":
  app.run(main)

