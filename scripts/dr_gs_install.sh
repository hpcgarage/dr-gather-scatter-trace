#!/bin/bash
# This script installs the dependencies for this tool and demonstrates how to test with the 
# Spatter benchmark suite
# Ex: ./scripts/dr_gs_install.sh

# Last updated: 8/1/2022

#DynamoRIO build - the minimum tested version is 8.0.18895 from 2021
DR_VERSION=9.0.19195

#Specify the top-level directory for the gather/scatter tool - we assume this script is run from that repo.
DR_GS_HOME=$PWD
#Out of source build folder
BUILD_HOME=$DR_GS_HOME/build


install_prereqs()
{
	#RHEL8
	sudo su -
	dnf -y install cmake.x86_64

	#Ubuntu 20.04
}

setup_dr()
{
	#Pull a recent release of DynamoRIO and untar it
	wget https://github.com/DynamoRIO/dynamorio/releases/download/cronbuild-${DR_VERSION}/DynamoRIO-Linux-${DR_VERSION}.tar.gz
	tar xzf DynamoRIO-Linux-${DR_VERSION}.tar.gz

	#Apply a patch to DynamoRIO to allow libraries to be publicly linked by cmake
	patch -u -b ${BUILD_HOME}/DynamoRIO-Linux-${DR_VERSION}/cmake/DynamoRIOConfig.cmake -i ${DR_GS_HOME}/dynamorio_cmake.patch
	
	export DYNAMORIO_ROOT=${BUILD_HOME}/DynamoRIO-Linux-${DR_VERSION}/	
}

test_spatter_client()
{
	git clone https://github.com/hpcgarage/spatter.git
	git checkout dynamorio_integration
}

#Create a separate build/test directory; clean an existing directory
mkdir -p $BUILD_HOME
cd $BUILD_HOME
rm -rf $BUILD_HOME/*

#install_prereqs
setup_dr
#build_dr_gs_client
#test_spatter_client

