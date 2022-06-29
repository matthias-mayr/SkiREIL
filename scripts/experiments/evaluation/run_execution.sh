#!/bin/bash
cwd=$(pwd)

# Make sure the symlink is re-established.
scripts/build/configure.sh

./deps/limbo/build/exp/skireil/src/dart/learning_skills_execution_simu "$@"
