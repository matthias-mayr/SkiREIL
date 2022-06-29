#!/bin/bash
cwd=$(pwd)

#PREFIX="valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --verbose --log-file=valgrind-leak.txt"

$cwd/scripts/build/configure.sh
source $cwd/scripts/paths.sh

if [ $# -ge 1 ]; then
    RUNS=$1
else
    RUNS=1
fi

for i in $(eval echo "{1..$RUNS}")
do
	echo "Experiment $i:"
	$PREFIX ./deps/limbo/build/exp/skireil/src/dart/learning_skills_simu --config $2
done
