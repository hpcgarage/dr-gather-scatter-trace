#!/bin/bash
# This script installs the dependencies for this tool and demonstrates how to test with the 
# Spatter benchmark suite
# Ex: ./scripts/dr_gs_install.sh

# Last updated: 8/1/2022

#DynamoRIO build - the minimum tested version is 8.0.18895 from 2021
#DR_VERSION=9.0.19195
DR_VERSION=8.0.18895

#Specify the top-level directory for the gather/scatter tool - we assume this script is run from that repo.
DR_GS_HOME=$PWD
#Out of source build folder
PROJ_HOME=$DR_GS_HOME/build


install_prereqs()
{
	#RHEL8
	sudo su -
	dnf -y install cmake.x86_64

	#Ubuntu 20.04
}

setup_dr()
{
	cd $PROJ_HOME
	
	#Pull a recent release of DynamoRIO and untar it
	wget https://github.com/DynamoRIO/dynamorio/releases/download/cronbuild-${DR_VERSION}/DynamoRIO-Linux-${DR_VERSION}.tar.gz
	tar xzf DynamoRIO-Linux-${DR_VERSION}.tar.gz

	#Apply a patch to DynamoRIO to allow libraries to be publicly linked by cmake
	patch -u -b ${PROJ_HOME}/DynamoRIO-Linux-${DR_VERSION}/cmake/DynamoRIOConfig.cmake -i ${DR_GS_HOME}/dynamorio_cmake.patch
	
}

build_dr_gs_client()
{
	cd $PROJ_HOME
	
	set -x
	#git clone https://github.com/hpcgarage/dr-gather-scatter-trace.git
	#cd dr-gather-scatter-trace
	cp -rf ../client .
	cd client
	mkdir -p client_build && cd client_build
	
	cmake -DDynamoRIO_DIR=${DYNAMORIO_ROOT}/cmake/ ..
	make -j
	set -x
}

#Run a validation test with the Spatter benchmark suite which can generate vectorized gather/scatter ops
test_spatter_client()
{
	cd $PROJ_HOME

	
	#git clone https://github.com/hpcgarage/spatter.git
	cd spatter
	#git checkout dynamorio_integration

	#chmod a+rx configure/configure_omp_intel_dynamorio
	./configure/configure_omp_intel_dynamorio
	
	cd build_omp_intel_dynamorio
	make -j

	#Run a simple Spatter test with two threads - you can tweak this to run with different thread counts
	NUM_THREADS=1
	export OMP_NUM_THREADS=$NUM_THREADS

	set -x
	${DYNAMORIO_ROOT}/bin64/drrun -noinject -c $DR_GS_CLIENT -- ./spatter -pUNIFORM:8:1 -t${NUM_THREADS}
	set -x
}

#Initialize key variables
set_env()
{
	export DYNAMORIO_ROOT=${PROJ_HOME}/DynamoRIO-Linux-${DR_VERSION}/	
	export DR_GS_CLIENT=${PROJ_HOME}/client/client_build/libgsclient.so
	export PATH=$PATH:${DYNAMORIO_ROOT}/bin64/	
}

clean_up()
{
	#Create a separate build/test directory; clean an existing directory
	cd $PROJ_HOME
	rm -rf $PROJ_HOME/*
}

mkdir -p $PROJ_HOME

#install_prereqs
set_env

#setup_dr
build_dr_gs_client
test_spatter_client

