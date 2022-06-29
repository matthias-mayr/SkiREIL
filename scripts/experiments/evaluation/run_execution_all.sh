#!/bin/bash
cwd=$(pwd)
# Exit on ctrl-c
trap "exit" INT
for i in {1..15}
do
    ./scripts/experiments/reset_iiwa.py $1$i
    ./scripts/experiments/evaluation/run_execution.sh skills --data $2 --config $3 $4 $5 $6 $7 $8
done
