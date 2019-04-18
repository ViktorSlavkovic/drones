#!/bin/bash

source run_util.sh

SP4_PROBLEM_TYPE_TEXT_PROTO=$'Nd_1:true,M_m:true'  
PROBLEM_FILE="/tmp/test_problem_file"
SOLUTION_FILE="/tmp/test_solution_file"

generate $PROBLEM_FILE $SP4_PROBLEM_TYPE_TEXT_PROTO --gen_max_coord=100 --gen_max_ipo=50

solve $PROBLEM_FILE $SOLUTION_FILE random 
grade $PROBLEM_FILE $SOLUTION_FILE 0
echo "Random solver: $grade_ret"

solve $PROBLEM_FILE $SOLUTION_FILE ecf 
grade $PROBLEM_FILE $SOLUTION_FILE 0
echo "ECF solver: $grade_ret"

solve $PROBLEM_FILE $SOLUTION_FILE sp4 
grade $PROBLEM_FILE $SOLUTION_FILE 0
echo "SP4 solver: $grade_ret"

