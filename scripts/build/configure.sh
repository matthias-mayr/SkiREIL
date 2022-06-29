#!/bin/bash
set -e
cwd=$(pwd)

# configure paths
source ./scripts/paths.sh

# go to limbo
cd deps/limbo
mkdir -p exp
cd exp
ln -s ../../../ skireil || true
# go back to limbo
cd ..

# configure
./waf configure --magnum_install_dir=${cwd}/install --magnum_install_dir=${cwd}/install --magnum_plugins_install_dir=${cwd}/install --magnum_integration_install_dir=${cwd}/install --nlopt=${cwd}/install --dart=${cwd}/install --robot_dart=${cwd}/install --exp skireil --cpp14
# --debug_build

# go back to original directory
cd ../..

# Sort out ros header files
folder=include/skireil/
msg_folder=include/skireil/ros_msgs/
cp ../../devel/include/skireil/ManageSkirosWorker.h ${msg_folder}
cp ../../devel/include/skireil/ManageSkirosWorkerRequest.h ${folder}
cp ../../devel/include/skireil/ManageSkirosWorkerResponse.h ${folder}
cp ../../devel/include/skireil/SkirosNextAction.h ${msg_folder}
cp ../../devel/include/skireil/SkirosNextActionRequest.h ${folder}
cp ../../devel/include/skireil/SkirosNextActionResponse.h ${folder}
cp ../../devel/include/skireil/ParamFloat.h ${folder}
cp ../../devel/include/skireil/ParamString.h ${folder}
cp ../../devel/include/skireil/ParamInt.h ${folder}
echo "If you see some copy errors for header files above, try to run the 'prepare_ros.sh' script and build the catkin workspace again."
