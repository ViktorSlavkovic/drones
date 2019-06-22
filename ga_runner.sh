#!/bin/bash

PROBLEM_FILE=mother_of_all_warehouses.in
ALLOC_FILE=mother_of_all_warehouses.alloc
GA_FORBIDDEN_STRATEGIES="0,3,4,6,7"
GA_STRATEGIES_PER_DRONE=100
GA_STRATEGY_REPEATS=20
GA_NUM_GENERATIONS=200
GA_POPULATION_SIZE=100
GA_SELECTION_SIZE=30
GA_CROSSINGOVER_SIZE=30
GA_MAX_TWISTS=5
GA_SAME_TWISTS=false
GA_KEEP_BOTH_CHILDREN=false
GA_MUTATION_SIZE=40
GA_MUTATION_PROB=0.15
GA_MUTATE_ALL_DRONES=false
NUM_THREADS=70

LOG_FILE=$(date +"%Y-%m-%d_%H-%M-%S").log

./main --problem_file=$PROBLEM_FILE --solver_type=ga                 \
       --ga_alloc_from_file=$ALLOC_FILE                              \
       --ga_forbidden_strategies=$GA_FORBIDDEN_STRATEGIES            \
       --ga_strategies_per_drone=$GA_STRATEGIES_PER_DRONE            \
       --ga_strategy_repeats=$GA_STRATEGY_REPEATS                    \
       --ga_num_generations=$GA_NUM_GENERATIONS                      \
       --ga_population_size=$GA_POPULATION_SIZE                      \
       --ga_selection_size=$GA_SELECTION_SIZE                        \
       --ga_crossingover_size=$GA_CROSSINGOVER_SIZE                  \
       --ga_max_twists=$GA_MAX_TWISTS                                \
       --ga_same_twists=$GA_SAME_TWISTS                              \
       --ga_keep_both_children=$GA_KEEP_BOTH_CHILDREN                \
       --ga_mutation_size=$GA_MUTATION_SIZE                          \
       --ga_mutate_all_drones=$GA_MUTATE_ALL_DRONES                  \
       --ga_mutation_prob=$GA_MUTATION_PROB                          \
       --ga_standalone_num_threads=$NUM_THREADS                      \
       2>&1 | tee $LOG_FILE

