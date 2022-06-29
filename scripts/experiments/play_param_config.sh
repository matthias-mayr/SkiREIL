#!/bin/bash
cwd=$(pwd)

scripts/build/configure.sh

./deps/limbo/build/exp/skireil/src/dart/play_learning_skills_graphic $1 $2 $3 $4 $5 -v
