#!/bin/bash

source run_util.sh

SP3_PROBLEM_TYPE_TEXT_PROTO=$'Nd_1:true,M_m:true,Nw_1:true,S0_inf:false,Np_1:true'  
PROBLEM_FILE="/tmp/test_problem_file"
SOLUTION_FILE="/tmp/test_solution_file"

generate $PROBLEM_FILE $SP3_PROBLEM_TYPE_TEXT_PROTO

solve $PROBLEM_FILE $SOLUTION_FILE random 
grade $PROBLEM_FILE $SOLUTION_FILE 0
echo "Random solver: $grade_ret"

solve $PROBLEM_FILE $SOLUTION_FILE ecf 
grade $PROBLEM_FILE $SOLUTION_FILE 0
echo "ECF solver: $grade_ret"

solve $PROBLEM_FILE $SOLUTION_FILE sp3 
grade $PROBLEM_FILE $SOLUTION_FILE 0
echo "SP3 solver: $grade_ret"

solve $PROBLEM_FILE $SOLUTION_FILE sp3 --sp3_do_tsp
grade $PROBLEM_FILE $SOLUTION_FILE 0
echo "SP3 solver with TSP: $grade_ret"

