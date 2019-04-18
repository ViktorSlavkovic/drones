#!/bin/bash

source run_util.sh

SP2_PROBLEM_TYPE_TEXT_PROTO=$'Nd_1:true,M_m:true,Np_1:true,No_1:true'  
PROBLEM_FILE="/tmp/test_problem_file"
SOLUTION_FILE="/tmp/test_solution_file"

generate $PROBLEM_FILE $SP2_PROBLEM_TYPE_TEXT_PROTO --gen_max_ipo=50

solve $PROBLEM_FILE $SOLUTION_FILE random 
grade $PROBLEM_FILE $SOLUTION_FILE 1
echo "Random solver: $grade_ret"

solve $PROBLEM_FILE $SOLUTION_FILE ecf 
grade $PROBLEM_FILE $SOLUTION_FILE 1
echo "ECF solver: $grade_ret"

solve $PROBLEM_FILE $SOLUTION_FILE sp2 
grade $PROBLEM_FILE $SOLUTION_FILE 1
echo "SP2 solver: $grade_ret"

