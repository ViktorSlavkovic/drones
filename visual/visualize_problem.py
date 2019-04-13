#!/usr/bin/python

import math
import sys
import matplotlib.pyplot as plt

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
product_weight = []
order_loc_x = []
order_loc_y = []
order_request = []
order_done = []

selected_warehouse = -1
selected_order = -1

def load_problem(file_path):
  global W, H, Nd, Np, Nw, No, T, M
  global warehouse_loc_x, warehouse_loc_y, warehouse_stock, warehouse_empty
  global order_loc_x, order_loc_y, order_request, order_done
  global product_weight

  print("Loading from: %s" % file_path)
  
  with open(file_path, "r") as fin:
    W, H, Nd, T, M = [int(x) for x in next(fin).split()]
    Np = int(next(fin).split()[0])
    product_weight = [int(x) for x in next(fin).split()]
    Nw = int(next(fin).split()[0])
    for w in range(Nw):
      x, y = [int(x) for x in next(fin).split()]
      warehouse_loc_x.append(x)
      warehouse_loc_y.append(y)
      warehouse_stock.append([int(x) for x in next(fin).split()])
      warehouse_empty.append(sum(warehouse_stock[w]) == 0)
    No = int(next(fin).split()[0])
    for o in range(No):
      x, y = [int(x) for x in next(fin).split()]
      order_loc_x.append(x)
      order_loc_y.append(y)
      next(fin)  # Eat one line - the one containing # of product items
      curr_order_request = [0 for x in range(Np)]
      for x in next(fin).split():
        curr_order_request[int(x)] += 1 
      order_request.append(curr_order_request)
      order_done.append(False)
  
  print("Loading done.")

def print_warehouse(w):
    fig = plt.gcf()
    plt.sca(fig.axes[1])
    plt.cla()
    plt.title("Warehouse %d" % w)
    plt.gca().invert_yaxis()
    plt.ylim(0, 75)
    plt.xlim(0, 500)
    plt.axis('off')
    if warehouse_empty[w]:
      plt.text(0, 74, 'Empty')
    else:
      i = 0
      for p in range(Np):
        if warehouse_stock[w][p] != 0:
          plt.text((i // 75) * 28, 74 - (i % 75),
                   '%d: %d' % (p, warehouse_stock[w][p]), fontsize=8)
          i += 1
    plt.draw()

def print_order(o):
    fig = plt.gcf()
    plt.sca(fig.axes[1])
    plt.cla()
    plt.title("Order %d" % o)
    plt.gca().invert_yaxis()
    plt.ylim(0, 75)
    plt.xlim(0, 500)
    plt.axis('off')
    if order_done[o]:
      plt.text(0, 74, 'Done')
    else:
      i = 0
      for p in range(Np):
        if order_request[o][p] != 0:
          plt.text((i // 75) * 28, 74 - (i % 75),
                   '%d: %d' % (p, order_request[o][p]), fontsize=8)
          i += 1
    plt.draw() 

def update_plot():
    fig = plt.gcf()
    plt.sca(fig.axes[0])
    plt.cla()
    plt.title("Map")
    wx = [warehouse_loc_x[w] for w in range(Nw) if not warehouse_empty[w]]
    wy = [warehouse_loc_y[w] for w in range(Nw) if not warehouse_empty[w]]
    swx = [] if selected_warehouse < 0 else [warehouse_loc_x[selected_warehouse]]
    swy = [] if selected_warehouse < 0 else [warehouse_loc_y[selected_warehouse]]
    ox = [order_loc_x[o] for o in range(No) if not order_done[o]]
    oy = [order_loc_y[o] for o in range(No) if not order_done[o]]
    sox = [] if selected_order < 0 else [order_loc_x[selected_order]]
    soy = [] if selected_order < 0 else [order_loc_y[selected_order]]
    plt.plot(wx, wy, 'ro', swx, swy, 'go', ox, oy, 'b.', sox, soy, 'g.')
    plt.draw()

    if selected_order >= 0: print_order(selected_order)
    if selected_warehouse >= 0: print_warehouse(selected_warehouse)

def onclick(event):
    if event.inaxes != plt.gcf().axes[0]: return

    global selected_order
    global selected_warehouse

    wdist = math.inf
    wbest = -1
    for w in range(Nw):
      if warehouse_empty[w]: continue
      dx = event.xdata - warehouse_loc_x[w]
      dy = event.ydata - warehouse_loc_y[w]
      dist = dx*dx + dy*dy
      if dist < wdist:
        wdist = dist
        wbest = w

    odist = math.inf
    obest = -1
    for o in range(No):
      if order_done[o]: continue
      dx = event.xdata - order_loc_x[o]
      dy = event.ydata - order_loc_y[o]
      dist = dx*dx + dy*dy
      if dist < odist:
        odist = dist
        obest = o

    if odist < wdist:
      selected_order = obest
      selected_warehouse = -1
    else:
      selected_order = -1
      selected_warehouse = wbest

    update_plot()

def problem_plot():
  fig = plt.figure() 
  
  fig.add_subplot(121)
  cid = fig.canvas.mpl_connect('button_press_event', onclick) 
  
  fig.add_subplot(122)
  plt.gca().invert_yaxis()
  plt.axis('off')
  
  update_plot()
  
  plt.show()

def main():
  if (len(sys.argv) != 2):
    print("Usage: ./visualize_problem.py <problem_file_path>")
    sys.exit(-1)
  load_problem(sys.argv[1])
  problem_plot()

if __name__== "__main__":
  main()

