#!/bin/bash
cwd=$(pwd)

# Make sure the symlink is re-established.
scripts/build/configure.sh

if [ $1 == "peg" ]; then
    ./deps/limbo/build/exp/skireil/src/dart/learning_skills_execution_simu --data res/evaluation_peg/ --config res/evaluation_peg/params.json -z
elif [ $1 == "push" ]; then
    ./deps/limbo/build/exp/skireil/src/dart/learning_skills_execution_simu --data res/evaluation_push/ --config res/evaluation_push/params.json -z
fi