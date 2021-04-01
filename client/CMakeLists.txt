
CMAKE_MINIMUM_REQUIRED (VERSION 3.10 FATAL_ERROR)

project(count_client)

add_definitions(-DUNIX)
add_definitions(-DX86_64)
add_definitions(-DSHOW_RESULTS)
add_definitions(-DDEBUG)

find_package(DynamoRIO)
if (NOT DynamoRIO_FOUND)
	message(FATAL_ERROR "fatal: could not find dynamorio")
endif()

#add_library(count_client SHARED "client.c;utils.c")
#add_library(count_client SHARED client.cpp)
add_library(count_client SHARED client.c)
set(DynamoRIO_FAST_IR ON)
configure_DynamoRIO_client(count_client)

use_DynamoRIO_extension(count_client drmgr)
use_DynamoRIO_extension(count_client drreg)
use_DynamoRIO_extension(count_client drutil)
use_DynamoRIO_extension(count_client drx)
use_DynamoRIO_extension(count_client droption)

#set_target_properties(myapp PROPERTIES RUNTIME_OUTPUT_DIRECTORY /nethome/vhuang31/myapp/build)

#target_link_libraries(myapp LINK_PUBLIC /nethome/vhuang31/DynamoRIO-Linux-8.0.0-1/samples/bin64/libmemtrace_x86_binary.so)
#configure_DynamoRIO_standalone(myapp)
