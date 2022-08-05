#!/bin/bash
set -e

CATKIN_WS=Workspaces/skireil_ws
ROS=true
COMPILE=true
BRANCH=main
ROS_DISTRO=melodic

POSITIONAL_ARGS=()

print_help() {
    echo "A script to build singularity containers for this algorithm."
    echo -e "\nOptions:"
    echo -e "\t -b|--branch <NAME>\tSpecify a branch to checkout. Default is 'master'."
    echo -e "\t -c|--skip-compile\tSkip algorithm compilation. Still compiles dependencies"
    echo -e "\t -h|--help\tPrint help"
    echo -e "\t -r|--skip-ros\tSkip ROS repository setup"
    exit 0
}

while [[ $# -gt 0 ]]; do
  case $1 in
    -c|--skip-compile)
      COMPILE=false
      shift # past argument
      ;;
    -r|--skip-ros)
      ROS=false
      shift # past argument
      ;;
    -b|--branch)
      BRANCH="$2"
      echo "Branch: " $BRANCH
      shift # past argument
      shift # past value
      ;;
    -h|--help)
      print_help
      exit 0
      ;;
    -*|--*)
      echo "Unknown option $1"
      exit 1
      ;;
    *)
      POSITIONAL_ARGS+=("$1") # save positional arg
      shift # past argument
      ;;
  esac
done
set -- "${POSITIONAL_ARGS[@]}" # restore positional parameters

# Workaround to make Gitlab CI use the correct public branch
if [ "$BRANCH" = "skireil_public" ] ; then
    BRANCH=main
fi

# System dependencies
sudo apt-get install -y tmux sshfs wget lsb-release gnupg software-properties-common

# Install ROS
if [ "$ROS" = true ] ; then
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    sudo apt-get update
    sudo apt-get install -y ros-$ROS_DISTRO-ros-base
fi
sudo apt-get install -y python-wstool python-catkin-tools python-rosdep python-pip python-rosinstall ros-$ROS_DISTRO-rosmon

# Catkin setup
mkdir -p $CATKIN_WS/src/skireil
cd $CATKIN_WS
git clone --recursive --branch $BRANCH https://github.com/matthias-mayr/SkiREIL.git src/skireil

# SkiREIL dependencies
cd src/skireil
scripts/installation/install_deps_all.sh

# Install some dependencies needed for the Cartesian impedance controller
cd ../..
mkdir depends
cd depends

git clone --recursive https://github.com/costashatz/SpaceVecAlg.git
cd SpaceVecAlg
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF ..
make -j
sudo make install
cd ../..

git clone --recursive https://github.com/costashatz/RBDyn.git
cd RBDyn
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF ..
make -j2
sudo make install
cd ../..

git clone --recursive https://github.com/costashatz/mc_rbdyn_urdf.git
cd mc_rbdyn_urdf
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF ..
make -j
sudo make install
cd ../..

cd ..

# Catkin setup
catkin init
catkin config --extend /opt/ros/$ROS_DISTRO
wstool init src src/skireil/res/setup.rosinstall
# Add skireil in the respective branch
echo "- git: {local-name: skireil, uri: 'git@git.cs.lth.se:robotlab/skireil.git', version: '$BRANCH'}" > /tmp/repo.rosinstall
wstool merge -t src /tmp/repo.rosinstall


sudo rosdep init || true
rosdep update
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y
catkin build
source devel/setup.bash

# Python dependencies
roscd skiros2
cd .. && python -m pip install -r requirements.txt --user

# Compile skireil
if [ "$COMPILE" = true ] ; then
    roscd skireil
    scripts/build/configure.sh
    scripts/build/compile.sh
fi
