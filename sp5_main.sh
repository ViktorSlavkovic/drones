#!/bin/bash

source run_util.sh

SP5_PROBLEM_TYPE_TEXT_PROTO=$'IPO_1:true'  
PROBLEM_FILE="/tmp/test_problem_file"
SOLUTION_FILE="/tmp/test_solution_file"

generate $PROBLEM_FILE $SP5_PROBLEM_TYPE_TEXT_PROTO --gen_max_coord=10 --gen_max_nd=3 --gen_max_np=3 --gen_max_nw=3 --gen_max_no=5 --gen_max_M=10 --gen_max_t=30

solve $PROBLEM_FILE $SOLUTION_FILE random 
grade $PROBLEM_FILE $SOLUTION_FILE 0
echo "Random solver: $grade_ret"

solve $PROBLEM_FILE $SOLUTION_FILE ecf 
grade $PROBLEM_FILE $SOLUTION_FILE 0
echo "ECF solver: $grade_ret"

solve $PROBLEM_FILE $SOLUTION_FILE sp5 
grade $PROBLEM_FILE $SOLUTION_FILE 0
echo "SP5 solver: $grade_ret"

