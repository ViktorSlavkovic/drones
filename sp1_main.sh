#!/bin/bash

source run_util.sh

SP1_PROBLEM_TYPE_TEXT_PROTO=$'Nd_1:true,M_m:true,Nw_1:true,S0_inf:true,Np_1:true,IPO_1:true'
PROBLEM_FILE="/tmp/test_problem_file"
SOLUTION_FILE="/tmp/test_solution_file"

generate $PROBLEM_FILE $SP1_PROBLEM_TYPE_TEXT_PROTO

solve $PROBLEM_FILE $SOLUTION_FILE random 
grade $PROBLEM_FILE $SOLUTION_FILE 0
echo "Random solver: $grade_ret"

solve $PROBLEM_FILE $SOLUTION_FILE ecf 
grade $PROBLEM_FILE $SOLUTION_FILE 0
echo "ECF solver: $grade_ret"

solve $PROBLEM_FILE $SOLUTION_FILE sp1 
grade $PROBLEM_FILE $SOLUTION_FILE 0
echo "SP1 solver: $grade_ret"

