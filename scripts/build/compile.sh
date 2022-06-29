#!/bin/bash
set -e
cwd=$(pwd)

# configure paths
source ./scripts/paths.sh

# go to limbo directory
cd deps/limbo

# Make the number of jobs an optional argument
if [ $# -eq 0 ]; then
    JOBS=6
else
    JOBS=$1
fi

# Warning for memory usage
echo "Compiling with $JOBS jobs. This will need about" `expr 4 \* $JOBS`"GB of memory. The number of jobs can be passed to this file as a parameter."

# Compile. Most options are set in 'configure.sh'.
./waf --exp skireil -j$JOBS

# go back to original directory
cd ../..

# Delete symlink since ROS can't work with it
# This one is usually created by the configure.sh script
#rm deps/limbo/exp/skireil

echo "Do not forget to source scripts/paths.sh to run an experiment"
