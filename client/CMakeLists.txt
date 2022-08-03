
CMAKE_MINIMUM_REQUIRED (VERSION 3.10 FATAL_ERROR)

project(gsclient)

add_definitions(-DUNIX -DX86_64 -DSHOW_RESULTS -DDEBUG -D__AVX512F__ -DVERBOSE)

find_package(DynamoRIO)
if (NOT DynamoRIO_FOUND)
	message(FATAL_ERROR "fatal: could not find dynamorio")
endif()

add_library(gsclient SHARED client.cpp)

#set(DynamoRIO_FAST_IR ON)
configure_DynamoRIO_client(gsclient)

use_DynamoRIO_extension(gsclient drmgr)
use_DynamoRIO_extension(gsclient drreg)
use_DynamoRIO_extension(gsclient drutil)
use_DynamoRIO_extension(gsclient drx)
use_DynamoRIO_extension(gsclient droption)
