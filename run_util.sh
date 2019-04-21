#!/bin/bash

function check {
  local val=$1
  local should_be=$2
  shift 2
  local message=$@
  if [ $val -ne $should_be ]; then
    echo $message 1>&2
    exit -1
  fi
}

function generate {
  local path=$1
  local problem_type=$2
  shift 2
  ./main \
    --problem_file=$path \
    --gen_problem \
    --write_gen_problem --gen_only \
    --gen_problem_type=$problem_type \
    $@ &> /dev/null
  check $? 0 "Failed to generate"
}

function solve {
  local in_path=$1
  local out_path=$2
  local solver=$3
  shift 3
  ./main \
    --problem_file=$in_path \
    --solution_file=$out_path \
    --solver_type=$solver \
    --check=false \
    $@ &> /dev/null
  check $? 0 "Failed to solve"
}

function grade {
  local in_path=$1
  local out_path=$2
  local use_strict_checker=$3  

  local score=$(./main \
    --problem_file=$in_path \
    --solution_file=$out_path \
    --read_solution 2>&1 | grep "TOTAL SCORE" | awk '{print $3}')
  local success=$?

  if [ $use_strict_checker -eq 1 ]; then 
    local strict_score=-1
    strict_score=$(checker/main $in_path $out_path 2>&1 | grep "TOTAL SCORE" | awk '{print $3}')
    local strict_success=$?
    check $success $strict_success "Regular checker exited with $success and strict checker exited  with $strict_success."
    check $score $strict_score "Regular checker gave $score and strict checker gave $strict_score."
 
    local visual_score=-1
    visual_score=$(visual/visualize_solution --problem_file=$in_path --solution_file=$out_path --just_calc 2>&1 | grep "TOTAL SCORE" | awk '{print $3}')
    local visual_success=$?
    check $success $visual_success "Regular checker exited with $success and visualization checker exited  with $visual_success."
    check $score $visual_score "Regular checker gave $score and visualization checker gave $visual_score."
  fi
  
  check $success 0 "Failed to grade (exit code $success)."

  grade_ret=$score
}
