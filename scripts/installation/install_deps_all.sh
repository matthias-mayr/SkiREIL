#!/bin/bash
set -e

if [ $# -eq 1 ]; then
    REINSTALL=true
    echo "Reinstalling libraries"
else
    REINSTALL=false
fi

function remove_build_dir_if_reinstall()
{
    if [ "$REINSTALL" = true ] ; then
        rm -rf build || true
    fi
}


OS=$(uname)
echo "Detected OS: $OS"

if [ $OS = "Darwin" ]; then
    echo "ERROR: OSX can only be used with install_deps_req.sh"
    exit 1
fi

# check if we have Ubuntu or not
distro_str="$(cat /etc/*-release | grep -s DISTRIB_ID)"
distro=$(echo $distro_str | cut -f2 -d'=')

if [ "$distro" != "Ubuntu" ]; then
    echo "ERROR: We need an Ubuntu system to use this script"
    exit 1
fi

sudo apt-get -qq update
# install Eigen 3, Boost and TBB
sudo apt-get --yes --force-yes install cmake libeigen3-dev libtbb-dev libboost-serialization-dev libboost-filesystem-dev libboost-test-dev libboost-program-options-dev libboost-thread-dev libboost-regex-dev libsdl2-dev

# install google tests for libcmaes
sudo apt-get --yes --force-yes install libgoogle-glog-dev libgflags-dev

# save current directory
cwd=$(pwd)
# create install dir
mkdir -p install

# configure paths
source ./scripts/paths.sh

# installing NLOpt
cd deps
wget http://members.loria.fr/JBMouret/mirrors/nlopt-2.4.2.tar.gz
tar -zxvf nlopt-2.4.2.tar.gz && cd nlopt-2.4.2
./configure -with-cxx --enable-shared --without-python --without-matlab --without-octave --prefix=${cwd}/install
make install
# go back to original directory
cd ../..

# get ubuntu version
version_str="$(cat /etc/*-release | grep -s DISTRIB_RELEASE)"
version=$(echo $version_str | cut -f2 -d'=')
major_version=$(echo $version | cut -f1 -d'.')
minor_version=$(echo $version | cut -f2 -d'.')

# if less than 14.04, exit
if [ "$(($major_version))" -lt "14" ]; then
    echo "ERROR: We need Ubuntu >= 14.04 for this script to work"
    exit 1
fi

# install DART dependencies
# if we have less than 16.04, we need some extra stuff
if [ "$(($major_version))" -lt "16" ]; then
    sudo apt-add-repository ppa:libccd-debs/ppa -y
    sudo apt-add-repository ppa:fcl-debs/ppa -y
fi
sudo apt-add-repository ppa:dartsim/ppa -y
sudo apt-get -qq update
sudo apt-get --yes --force-yes install build-essential pkg-config libassimp-dev libccd-dev libfcl-dev
sudo apt-get --yes --force-yes install libnlopt-dev libbullet-dev libtinyxml-dev libtinyxml2-dev liburdfdom-dev liburdfdom-headers-dev libxi-dev libxmu-dev freeglut3-dev libopenscenegraph-dev
# install DART
cd deps/dart
remove_build_dir_if_reinstall
mkdir -p build && cd build
cmake -DDART_ENABLE_SIMD=ON -DBUILD_PYTHON=ON -DCMAKE_INSTALL_PREFIX=${cwd}/install ..
make -j4
make install
# go back to original directory
cd ../../..

# just as fail-safe
sudo ldconfig

# configure paths to find DART related libraries properly
source ./scripts/paths.sh

# Corrade - ineffective as of now
cd deps/corrade
remove_build_dir_if_reinstall
mkdir build && cd build
# Fix: Install globally, because magnum can not find it otherwise
#cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${cwd}/install ..
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
sudo make install
cd ../../..

sudo apt-get install -y --no-install-recommends libglfw3-dev libglfw3 libopenal-dev libjpeg-dev libpng-dev
cd deps/magnum
remove_build_dir_if_reinstall
mkdir build && cd build

cmake -DCMAKE_BUILD_TYPE=Release -DWITH_AUDIO=ON -DWITH_DEBUGTOOLS=ON -DWITH_GL=ON -DWITH_MESHTOOLS=ON -DWITH_PRIMITIVES=ON -DWITH_SCENEGRAPH=ON -DWITH_SHADERS=ON -DWITH_TEXT=ON -DWITH_TEXTURETOOLS=ON -DWITH_TRADE=ON -DWITH_GLFWAPPLICATION=ON -DWITH_WINDOWLESSGLXAPPLICATION=ON -DWITH_OPENGLTESTER=ON -DWITH_ANYAUDIOIMPORTER=ON -DWITH_ANYIMAGECONVERTER=ON -DWITH_ANYIMAGEIMPORTER=ON -DWITH_ANYSCENEIMPORTER=ON -DWITH_MAGNUMFONT=ON -DWITH_OBJIMPORTER=ON -DWITH_TGAIMPORTER=ON -DWITH_WAVAUDIOIMPORTER=ON -DCMAKE_INSTALL_PREFIX=${cwd}/install .. # this will enable almost all features of Magnum that are not necessarily needed for robot_dart (please refer to the documentation of Magnum for more details on selecting only the ones that you need)
make -j
make install
cd ../../..

# install MagnumPlugins
cd deps/magnum-plugins
remove_build_dir_if_reinstall
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DWITH_ASSIMPIMPORTER=ON -DWITH_DDSIMPORTER=ON -DWITH_JPEGIMPORTER=ON -DWITH_OPENGEXIMPORTER=ON -DWITH_PNGIMPORTER=ON -DWITH_TINYGLTFIMPORTER=ON -DWITH_STBTRUETYPEFONT=ON -DCMAKE_INSTALL_PREFIX=${cwd}/install ..
make -j4
make install
cd ../../..

# install MagnumIntegration
cd deps/magnum-integration
remove_build_dir_if_reinstall
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DWITH_DART=ON -DWITH_EIGEN=ON -DDART_INCLUDE_DIRS=${cwd}/install -DCMAKE_INSTALL_PREFIX=${cwd}/install ..
make -j4
make install
cd ../../..

sudo ldconfig

# install robot_dart
cd deps/robot_dart
./waf configure --dart=${cwd}/install --magnum_install_dir=${cwd}/install --magnum_plugins_install_dir=${cwd}/install --magnum_integration_install_dir=${cwd}/install --prefix=${cwd}/install
./waf
./waf install
# go back to original directory
cd ../..

# Make sure that catkin ignores DART
touch deps/dart/CATKIN_IGNORE