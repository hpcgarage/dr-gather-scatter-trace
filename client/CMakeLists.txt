
CMAKE_MINIMUM_REQUIRED (VERSION 3.10 FATAL_ERROR)

project(count_client)

add_definitions(-DUNIX -DX86_64 -DSHOW_RESULTS -DDEBUG -D__AVX512F__ -DVERBOSE)

find_package(DynamoRIO)
if (NOT DynamoRIO_FOUND)
	message(FATAL_ERROR "fatal: could not find dynamorio")
endif()

add_library(count_client SHARED client.c client_tools.h avx512ctx-shared.h detach_state_shared.h)
#set(DynamoRIO_FAST_IR ON)
configure_DynamoRIO_client(count_client)

use_DynamoRIO_extension(count_client drmgr)
use_DynamoRIO_extension(count_client drreg)
use_DynamoRIO_extension(count_client drutil)
use_DynamoRIO_extension(count_client drx)
use_DynamoRIO_extension(count_client droption)
