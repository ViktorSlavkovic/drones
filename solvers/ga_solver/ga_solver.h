#ifndef SOLVERS_GA_SOLVER_GA_SOLVER_H_
#define SOLVERS_GA_SOLVER_GA_SOLVER_H_

#include <functional>
#include <map>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include "problem.pb.h"
#include "solvers/problem_solver.h"
#include "solvers/util/allocator.h"

namespace drones {

class GaSolver : public ProblemSolver {
 public:
  explicit GaSolver(const Problem& problem);
  std::unique_ptr<Solution> Solve() override;
  bool CanSolve(const ProblemType& problem_type) const override { return true; }

 protected:
  // The commands are generated through transactions. Each transaction is bound
  // to a single drone and at most one transaction can be happening for a drone
  // at each time. This structure holds the data needed to manage a transaction.
  struct CommandTransactionData {
    // Current drone location, it starts as drone_loction_ of the drone and
    // then changes as the transaction advances. On transation commit, it is
    // used to update drone_location_.
    int curr_drone_location;
    // Same as the above, just for drone_busy_until_.
    int curr_drone_busy_until;
    // Number of commands generated for the drone since the transaction has
    // started.
    int gen_commands;
  };

  // Represents all solution-specific, thread-local data.
  struct SolutionBuilder {
    using DroneStrategies = std::vector<std::vector<uint8_t>>;

    explicit SolutionBuilder(const GaSolver& solver);

    // The main solver - use this to access the constants (e.g.
    // kClosestWarehouses).
    const GaSolver& solver;
    // Just ashortcut to solver.problem_;
    const Problem& problem;
    // Location of each drone as a location code (warehouses are [0, Nw-1] and
    // orders are [Nw, Nw + No - 1]).
    std::vector<int> drone_location;
    // Last moment at which the drone is busy.
    std::vector<int> drone_busy_until;
    // For each warehouse, it stores what it has or will have for a certain
    // order. For each (w, o) pair it stores additions to the warehouse mapped
    // by the moments at which they occur. Structure: [w][o][t][p] -> n Keep
    // compact!
    std::map<int, std::map<int, std::map<int, std::map<int, int>>>>
        pending_warehouses;
    // For each order, it stores what should be deliverd from which warehouse.
    // Structure: [o][w][p] -> n
    // Keep compact!
    std::map<int, std::map<int, std::map<int, int>>> pending_orders;
    // Total pending weight for each order.
    std::vector<int> pending_orders_weight;
    // Maps drones to their ongoing transactions.
    std::map<int, CommandTransactionData> transactions;
    // The solution being built.
    std::unique_ptr<Solution> solution;
    // Latest delivery time for each order.
    std::vector<int> latest_delivery_time;
    // Score of the solution being build.
    int score;

    // Returns pair (int - wait time, std::map<int, int> inventory map) which
    // says what can be taken upon arrival and how much do we need to wait.
    std::pair<int, std::map<int, int>> CanTake(int arrival, int w, int o);
    // Takes the specified inventory (what) from the specified warehouse (w) for
    // the specified order (o). This doesn't generate any commands, it just does
    // the book-keeping. It's assumed that what is a subset of what can be taken
    // as returned by CanTake().
    void Take(int arrival, int w, int o, std::map<int, int> what,
              bool manual_relax_order = false);
    // Gives the specified inventory (what) to the specified warehouse (w) for
    // the specified order (o). This doesn't generate any commands, it just does
    // the book-keeping. It's assumed that what is the same what that has been
    // taken before by Take().
    void Give(int arrival, int w, int o, const std::map<int, int>& what);
    // Delivers the specified inventory (what) to the specified order (o) from
    // the specified warehouse (w). This doesn't generate any commands, it just
    // does the book-keeping. It's assumed that what is the same what that has
    // been taken befire by Take() and that what is a subset of what is left for
    // that order. Returns true if the order gets completed.
    bool RelaxOrder(int w, int o, std::map<int, int> what);

    // Starts a transaction for the drone d. At most one transaction can be
    // happening for each drone, so it dies if there's already  a transaction
    // happening.
    void CommandTransactionStart(int d);
    // Rolls back the transaction started for the drone d earlier by calling
    // CommandTransactionStart(d). Dies if there isn't a transaction happening
    // for d. This ends the transaction.
    void CommandTransactionRollBack(int d);
    // Commits the transaction started for the drone d earlier by calling
    // CommandTransactionStart(d). Dies if there isn't a transaction happening
    // for d. This ends the transaction.
    void CommandTransactionCommit(int d);
    // The following methods generate the respective commands within a
    // transaction. True is returned on success. If a failure occurs, roll-back
    // the transaction.
    bool Load(int d, int w, const std::map<int, int>& what, int wait_time);
    bool Unload(int d, int w, const std::map<int, int>& what);
    bool Deliver(int d, int o, const std::map<int, int>& what);
    bool Wait(int d, int wait_time);

    // This performs a combo transation: Load + Unload. It's a self-contained
    // transatction - a transaction is started and the return value tells
    // whether it's successfully commited (true) or rolled-back (false).
    bool ComboMoveAtomic(int d, int w_from, int w_to, int o);
    // Same as the above, only for: Load + Deliver.
    bool ComboDeliverAtomic(int d, int w, int o);

    // The following methods represent drone strategies:
    // 0: Go to the closest warehouse that has something to give, take something
    //    from it to the closest warehouse of an order. The order is chosen so
    //    that the inter-warehouse distance is minimal. The warehouses have to
    //    be different.
    //    TODO: What if the order is closer to the source warehose than it's
    //          closest warehouse.
    //    Type: MOVE
    bool Strategy0(int d);
    // 1: Go to the closest warehouse that has something to give and take
    //    something to the closest order for which there's something in that
    //    warehouse.
    //    Type: DELIVER
    bool Strategy1(int d);
    // 2: Find the order which is closest to completion, find the warehouse from
    //    which you'll deliver fastest and deliver.
    //    Type: DELIVER
    bool Strategy2(int d);
    // 3: Find the order which is closest to completion, move something closer
    //    from. Choose src and dst warehouses so that dst is closer than src
    //    and is in top kNumConsideredWarehouses (see the impl.) closest
    //    warehouses to the order and so that overall moving time is minimized.
    //    Type: MOVE
    bool Strategy3(int d);
    // 4: Like 3, but take to the closest warehouse only.
    //    Type: MOVE
    bool Strategy4(int d);
    // 5: Fastest delivery - like 2, only the order is not fixed.
    //    Type: DELIVER
    bool Strategy5(int d);
    // 6: Fastest move to a closer warehouse in the top kNumConsideredWarehouses
    //    (see impl.) closest warehouses to an order. Like 3, only the order is
    //    not fixed.
    //    Type: MOVE
    bool Strategy6(int d);
    // 7: Fastest move to the closest warehouse of an order. Like 4, only the
    //    order is not fixed.
    //    Type: MOVE
    bool Strategy7(int d);

    // Builds a solution using the provided drone strategies. For each drone,
    // there's a vector of strategies that are executed circularly.
    void Build(const DroneStrategies& drone_strategies,
               bool log_success_ratios = false);
  };

  // GA individual representation, less readable, easier to manipulate from the
  // GA's view. It's an alternative to SolutionBuilder::DroneStrategies, which
  // is far more readable.
  using Individual = std::vector<std::vector<bool>>;

  // Sorted (ascending by distance from the order) warehouse indices for each
  // order.
  const std::vector<std::vector<int>> kClosestWarehouses;
  // Product weights map as required by the splitter.
  const std::map<int, int> kProductWeights;
  // The alloc - which warehouse gives what exactly to which order.
  const util::Allocator::Alloc kAlloc;
  // Enumerates the implemented strategies to make it easier to call them by
  // number.
  const std::vector<std::function<bool(SolutionBuilder*, int)>> kStrategies;
  // The number of bits required to store an index of kStrategies.
  const int kLog2NumStrategies;

  // Calculates closest warehouses for each order (see kClosestWarehouses).
  static std::vector<std::vector<int>> CalcClosestWarehouses(
      const Problem& problem);
  // Fills in the product weights map (see kProductWeights).
  static std::map<int, int> PrepareProductWeights(const Problem& problem);
  // Creates an alloc using util::Allocator and does verification. Dies on
  // invalid alloc.
  static util::Allocator::Alloc AllocateOrDie(const Problem& problem);
  // Creates a vector of filtered-out (FLAGS_ga_forbidden_strategies)
  // strategies. Used to initialize kStrategies.
  static std::vector<std::function<bool(SolutionBuilder*, int)>>
  FilterStrategies(GaSolver* ga_solver);

  // Random engine used to generate all random numbers in this solver.
  std::default_random_engine random_engine_;

  // Convert Individual to SolutionBuilder::DroneStrategies.
  SolutionBuilder::DroneStrategies IndividualToDroneStrategies(
      const Individual& individual);
  // Generates a single individual in the GA's population.
  Individual GenerateIndividual();
  // Do a single evaluation.
  int Eval(const Individual& individual);
  // Runs the Genetic Algorithm, and stores the best solution in solution_.
  Individual RunGA();
};
}  // namespace drones

#endif  // SOLVERS_GA_SOLVER_GA_SOLVER_H_
