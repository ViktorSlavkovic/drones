#include <bits/stdc++.h>

using namespace std;

int W, H, Nd, Np, Nw, No, T, M;
vector<int> product_weight;
vector<pair<int, int>> warehouse_loc;
vector<unordered_map<int, int>> warehouse_stock;
vector<pair<int, int>> order_loc;
vector<unordered_map<int, int>> order_request;
vector<pair<int, int>> drone_loc;
vector<unordered_map<int, int>> drone_inventory;
vector<int> drone_weight;

struct DroneCommand {
  int d;
  char type;
  union {
    struct {
      int w;
      int p;
      int n;
    } L, U;
    struct {
      int o;
      int p;
      int n;
    } D;
    struct {
      int d;
    } W;
  } spec;
};

vector<queue<DroneCommand>> drone_commands;

multimap<int, int> drone_schedule;                      // t -> d
multimap<int, tuple<int, int, int>> unload_schedule;    // t -> (w, p, n)
multimap<int, tuple<int, int, int>> delivery_schedule;  // t -> (o, p, n)

int64_t score;

bool first_int(const string& path) {
  ifstream fin(path);
  assert(fin.good());
  string s;
  fin >> s;
  if (s.empty()) {
    fin.close();
    return false;
  }
  for (char c : s) {
    if (!isdigit(c)) {
      fin.close();
      return false;
    }
  }
  fin.close();
  return true;
}

void LoadProblem(const string& path) {
  cout << "Loading problem file: " << path << endl;
  ifstream fin(path);
  assert(fin.good());

  fin >> W >> H >> Nd >> T >> M;
  assert(W >= 1);
  assert(W <= 100000);
  assert(H >= 1);
  assert(H <= 100000);
  assert(Nd >= 1);
  assert(Nd <= 1000);
  assert(T >= 1);
  assert(T <= 1000000);
  assert(M >= 1);
  assert(M <= 10000);

  fin >> Np;
  assert(Np >= 1);
  assert(Np <= 10000);
  product_weight.resize(Np);
  for (int p = 0; p < Np; p++) {
    fin >> product_weight[p];
    assert(product_weight[p] >= 1);
    assert(product_weight[p] <= M);
  }

  fin >> Nw;
  assert(Nw >= 1);
  assert(Nw <= 10000);
  warehouse_loc.resize(Nw);
  warehouse_stock.resize(Nw);
  for (int w = 0; w < Nw; w++) {
    int x, y;
    fin >> x >> y;
    warehouse_loc[w] = {x, y};
    assert(x >= 0);
    assert(x < W);
    assert(y >= 0);
    assert(y < H);
    for (int p = 0; p < Np; p++) {
      int stock;
      fin >> stock;
      assert(stock >= 0);
      assert(stock <= 10000);
      warehouse_stock[w][p] = stock;
    }
  }

  fin >> No;
  assert(No >= 1);
  assert(No <= 10000);
  order_loc.resize(No);
  order_request.resize(No);
  for (int o = 0; o < No; o++) {
    int x, y;
    fin >> x >> y;
    order_loc[o] = {x, y};
    assert(x >= 0);
    assert(x < W);
    assert(y >= 0);
    assert(y < H);
    int num_items;
    fin >> num_items;
    assert(num_items >= 1);
    assert(num_items <= 10000);
    while (num_items--) {
      int p;
      fin >> p;
      assert(p >= 0);
      assert(p < Np);
      order_request[o][p]++;
    }
  }

  drone_loc.resize(Nd, warehouse_loc[0]);
  drone_weight.resize(Nd, 0);
  drone_inventory.resize(Nd);

  fin.close();

  cout << "Problem loaded successfully." << endl;
  cout << "Nd = " << Nd << endl;
  cout << "Np = " << Np << endl;
  cout << "Nw = " << Nw << endl;
  cout << "No = " << No << endl;
}

void LoadSolution(const string& path) {
  cout << "Loading solution file: " << path << endl;
  ifstream fin(path);
  assert(fin.good());

  drone_commands.resize(Nd);
  int Q;
  fin >> Q;
  assert(Q >= 0);
  assert(Q <= Nd * T);
  cout << "  Loading " << Q << " commands." << endl;
  const set<char> valid_commands = {'L', 'U', 'D', 'W'};
  while (Q--) {
    int d;
    fin >> d;
    assert(d >= 0);
    assert(d < Nd);
    char type;
    {
      string ts;
      fin >> ts;
      assert(ts.size() == 1);
      type = ts[0];
    }
    assert(valid_commands.count(type) == 1);
    DroneCommand cmd{.d = d, .type = type};
    switch (type) {
      case 'L': {
        fin >> cmd.spec.L.w >> cmd.spec.L.p >> cmd.spec.L.n;
        assert(cmd.spec.L.w >= 0);
        assert(cmd.spec.L.w < Nw);
        assert(cmd.spec.L.p >= 0);
        assert(cmd.spec.L.p < Np);
        assert(cmd.spec.L.n >= 1);
        break;
      }
      case 'U': {
        fin >> cmd.spec.U.w >> cmd.spec.U.p >> cmd.spec.U.n;
        assert(cmd.spec.U.w >= 0);
        assert(cmd.spec.U.w < Nw);
        assert(cmd.spec.U.p >= 0);
        assert(cmd.spec.U.p < Np);
        assert(cmd.spec.U.n >= 1);
        break;
      }
      case 'D': {
        fin >> cmd.spec.D.o >> cmd.spec.D.p >> cmd.spec.D.n;
        assert(cmd.spec.D.o >= 0);
        assert(cmd.spec.D.o < No);
        assert(cmd.spec.D.p >= 0);
        assert(cmd.spec.D.p < Np);
        assert(cmd.spec.D.n >= 1);
        break;
      }
      case 'W': {
        fin >> cmd.spec.W.d;
        assert(cmd.spec.W.d >= 1);
        assert(cmd.spec.W.d <= T);
      }
    }
    drone_commands[d].push(cmd);
  }

  fin.close();
  cout << "Solution loaded successfully." << endl;
}

int next_relevant_moment() {
  int res = numeric_limits<int>::max();
  if (!drone_schedule.empty()) res = min(res, drone_schedule.begin()->first);
  if (!unload_schedule.empty()) res = min(res, unload_schedule.begin()->first);
  if (!delivery_schedule.empty())
    res = min(res, delivery_schedule.begin()->first);
  if (res == numeric_limits<int>::max()) res = -1;
  return res;
}

int command_duration(const DroneCommand& cmd) {
  int d = cmd.d;
  pair<int, int> dest_loc;
  switch (cmd.type) {
    case 'L': {
      dest_loc = warehouse_loc[cmd.spec.L.w];
      break;
    }
    case 'U': {
      dest_loc = warehouse_loc[cmd.spec.U.w];
      break;
    }
    case 'D': {
      dest_loc = order_loc[cmd.spec.D.o];
      break;
    }
    case 'W':
      return cmd.spec.W.d;
  }
  double dx = dest_loc.first - drone_loc[d].first;
  double dy = dest_loc.second - drone_loc[d].second;
  return static_cast<int>(ceil(sqrt(dx * dx + dy * dy))) + 1;
}

void Simulate() {
  cout << "Simulate()" << endl;
  for (int d = 0; d < Nd; d++) drone_schedule.insert({0, d});

  int t;
  while ((t = next_relevant_moment()) >= 0) {
    assert(t < T);
    // (1) Handle drones.
    for (const auto& event : drone_schedule) {
      if (event.first != t) break;
      int d = event.second;

      if (drone_commands[d].empty()) continue;
      auto cmd = drone_commands[d].front();
      drone_commands[d].pop();

      int cmd_duration = command_duration(cmd);
      int finish_time = t + cmd_duration - 1;
      assert(finish_time < T);
      if (finish_time < T - 1) drone_schedule.insert({finish_time + 1, d});

      switch (cmd.type) {
        case 'L': {
          int w = cmd.spec.L.w;
          int p = cmd.spec.L.p;
          int n = cmd.spec.L.n;
          assert(warehouse_stock[w][p] >= n);
          warehouse_stock[w][p] -= n;
          assert(M - drone_weight[d] >= n * product_weight[p]);
          drone_weight[d] += n * product_weight[p];
          drone_inventory[d][p] += n;
          drone_loc[d] = warehouse_loc[w];
          break;
        }
        case 'U': {
          int w = cmd.spec.U.w;
          int p = cmd.spec.U.p;
          int n = cmd.spec.U.n;
          assert(drone_weight[d] >= n * product_weight[p]);
          drone_weight[d] -= n * product_weight[p];
          assert(drone_inventory[d][p] >= n);
          drone_inventory[d][p] -= n;
          unload_schedule.insert({finish_time, {w, p, n}});
          drone_loc[d] = warehouse_loc[w];
          break;
        }
        case 'D': {
          int o = cmd.spec.D.o;
          int p = cmd.spec.D.p;
          int n = cmd.spec.D.n;
          assert(drone_weight[d] >= n * product_weight[p]);
          drone_weight[d] -= n * product_weight[p];
          assert(drone_inventory[d][p] >= n);
          drone_inventory[d][p] -= n;
          delivery_schedule.insert({finish_time, {o, p, n}});
          drone_loc[d] = order_loc[o];
          break;
        }
        case 'W': {
          break;
        }
      }
    }
    drone_schedule.erase(t);

    // (2) Handle unload.
    for (const auto& event : unload_schedule) {
      if (event.first != t) break;
      int w = get<0>(event.second);
      int p = get<1>(event.second);
      int n = get<2>(event.second);
      warehouse_stock[w][p] += n;
    }
    unload_schedule.erase(t);

    // (3) Handle delivery.
    for (const auto& event : delivery_schedule) {
      if (event.first != t) break;
      int o = get<0>(event.second);
      int p = get<1>(event.second);
      int n = get<2>(event.second);
      assert(order_request[o][p] >= n);
      order_request[o][p] -= n;
      if (order_request[o][p] == 0) {
        order_request[o].erase(p);
        if (order_request[o].empty()) {
          int64_t pts = static_cast<int64_t>(ceil((100.0 * (T - t)) / T));
          cout << "Order " << o << " completed at: " << t << " -> " << pts
               << endl;
          score += pts;
        }
      }
    }
    delivery_schedule.erase(t);
  }
}

// Usage: ./whatever <problem_file_path> <solution_file_path>
int main(int argc, char* argv[]) {
  assert(argc == 3);
  assert(first_int(argv[1]));
  LoadProblem(argv[1]);
  assert(first_int(argv[2]));
  LoadSolution(argv[2]);
  Simulate();
  cout << "TOTAL SCORE: " << score << endl;
  return 0;
}
