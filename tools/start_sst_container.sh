#!/bin/bash
# This script opens a terminal to a SST container and maps a host directory into the container workspace.

HOSTPATH=/localscratch/jyoung9/sstbuild
#Docker image name or ID from `docker image ls`
DOCKERIMAGE=c6bf4f8d2b07
docker run -i -t -v $HOSTPATH:/build/workspace -e LOCAL_USER_ID=`id -u $USER` $DOCKERIMAGE /bin/bash
