#!/bin/bash
cwd=$(pwd)

# configure paths
# configure LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${cwd}/libcmaes/lib/python2.7/dist-packages/:${cwd}/install/lib:${LD_LIBRARY_PATH}

# configure PYTHONPATH
export PYTHONPATH=${cwd}/libcmaes/lib/python2.7/dist-packages/:${PYTHONPATH}

# go to limbo
cd limbo
mkdir -p exp
cd exp
ln -s ../../ blackdrops
# go back to limbo
cd ..

# save current directory
./waf configure --libcmaes=${cwd}/install --dart=${cwd}/install --robot_dart=${cwd}/install --exp blackdrops

# go back to original directory
cd ..