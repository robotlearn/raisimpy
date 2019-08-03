#!/bin/sh
# This bash script install the raisim libraries (raisimLib, raisimOgre, raisimGym) and 
# the python wrapper raisimpy. Note that is based on the instructions provided in these 
# repositories. 
#
# WARNING: Do not run this script without reading it first as it can mess up your 
# configuration. Check each line in this script and run them if you are sure.
# 
# repos:
# - raisimLib: https://github.com/leggedrobotics/raisimLib
# - raisimOgre: https://github.com/leggedrobotics/raisimOgre
# - raisimGym: https://github.com/leggedrobotics/raisimGym
# - raisimpy: https://github.com/robotlearn/raisimpy

# Define few variables
currentdir=$PWD

# Check if user is root/running with sudo
if [ `whoami` != root ]; then
    echo "Please run this script with sudo"
    exit
fi

# check the first argument which must be the workspace directory
if [ -z $1 ]
then
    echo "Specify the absolute location where to install the raisim libraries."
    exit
fi

# check the second argument which must be the python version
if [ -z $2 ]
then
    echo "Specify the python version you wish to use for raisimpy (e.g. 3.5)."
    exit
fi

# define variables
RAISIM_WORKSPACE=$1
RAISIM_BUILD=$1/raisim_build/
PYTHON_VERSION=$2

# move to that directory
cd; cd $RAISIM_WORKSPACE
mkdir $RAISIM_BUILD

# install dependencies

# install Eigen
sudo apt-get install libeigen-dev

# install the lastest version of g++
sudo apt-get install -y software-properties-common
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install g++-8 gcc-8

# install CMake
export cmake_version=3.14
export cmake_build=5
wget https://cmake.org/files/v$cmake_version/cmake-$cmake_version.$cmake_build.tar.gz
tar -xzvf cmake-$cmake_version.$cmake_build.tar.gz
cd cmake-$cmake_version.$cmake_build/
./bootstrap
make -j4
sudo make install
cd $RAISIM_WORKSPACE

# install ffmpeg
sudo add-apt-repository ppa:jonathonf/ffmpeg-4
sudo apt-get update
sudo apt-get install ffmpeg

# install dependencies for Ogre
sudo apt-get install libgles2-mesa-dev libxt-dev libxaw7-dev libsdl2-dev libzzip-dev libfreeimage-dev libfreetype6-dev libpugixml-dev

# install dependencies for raisimGym
sudo apt-get install libyaml-cpp-dev


# install raisim

# clone the repositories
git clone https://github.com/leggedrobotics/raisimLib
git clone https://github.com/leggedrobotics/raisimOgre
git clone https://github.com/leggedrobotics/raisimGym
git clone https://github.com/robotlearn/raisimpy

# build raisimLib
cd $RAISIM_WORKSPACE/raisimLib
mkdir build && cd build && cmake .. -DCMAKE_INSTALL_PREFIX=$RAISIM_BUILD && make install -j4

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$RAISIM_BUILD/lib


# build raisimOgre

# install locally Ogre
cd $RAISIM_WORKSPACE
git clone https://github.com/leggedrobotics/ogre.git
cd ogre
git checkout raisimOgre
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$RAISIM_BUILD -DOGRE_BUILD_COMPONENT_BITES=ON -OGRE_BUILD_COMPONENT_JAVA=OFF -DOGRE_BUILD_DEPENDENCIES=OFF -DOGRE_BUILD_SAMPLES=False
make install -j4

# build raisimOgre
cd $RAISIM_WORKSPACE/raisimOgre
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$RAISIM_BUILD -DCMAKE_INSTALL_PREFIX=$RAISIM_BUILD -DRAISIM_OGRE_EXAMPLES=True
make install -j4


# build raisimGym
cd $RAISIM_WORKSPACE/raisimGym

# install pybind11
cd $RAISIM_WORKSPACE
git clone https://github.com/pybind/pybind11.git
cd pybind11 && git checkout v2.2.4 && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$RAISIM_BUILD -DPYBIND11_TEST=OFF -DPYBIND11_PYTHON_VERSION=$PYTHON_VERSION
make install -j4



# build raisimpy
cd $RAISIM_WORKSPACE/raisimpy
cp -r extras $RAISIM_BUILD/include/ode/
mkdir build && cd build
cmake -DPYBIND11_PYTHON_VERSION=$PYTHON_VERSION -DCMAKE_PREFIX_PATH=$RAISIM_BUILD -DCMAKE_INSTALL_PREFIX=$RAISIM_BUILD ..
make install -j4


# write variables to export in the ~/.bashrc file, and source it
echo "" >> ~/.bashrc
echo "# Export environment variables for Raisim" >> ~/.bashrc
echo "export RAISIM_WORKSPACE=${RAISIM_WORKSPACE}" >> ~/.bashrc
echo "export RAISIM_BUILD=${RAISIM_BUILD}" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\$RAISIM_BUILD/lib" >> ~/.bashrc
# source .bashrc

# go back to the workspace directory
cd $RAISIM_WORKSPACE
