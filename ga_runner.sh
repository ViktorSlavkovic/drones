#!/bin/bash

PROBLEM_FILE=busy_day.in
ALLOC_FILE=busy_day.alloc
GA_FORBIDDEN_STRATEGIES=100
GA_STRATEGIES_PER_DRONE=1000
GA_STRATEGY_REPEATS=3
GA_NUM_GENERATIONS=200
GA_POPULATION_SIZE=200
GA_SELECTION_SIZE=80
GA_CROSSINGOVER_SIZE=60
GA_MUTATION_SIZE=60
GA_MUTATION_PROB=0.05
GA_MAX_TWISTS=5
GA_MUTATE_ALL_DRONES=false
NUM_THREADS=96

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
       --ga_mutation_size=$GA_MUTATION_SIZE                          \
       --ga_mutate_all_drones=$GA_MUTATE_ALL_DRONES                  \
       --ga_max_twists=$GA_MAX_TWISTS                                \
       --ga_mutation_prob=$GA_MUTATION_PROB                          \
       --ga_standalone_num_threads=$NUM_THREADS                      \
       2>&1 | tee $LOG_FILE

