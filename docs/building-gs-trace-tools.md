These instructions were tested on an Ubuntu 18.04 server with Cascade Lake CPU.

We use three different repositories to demonstrate how to build this trace tool for 1) validation with Spatter 
and 2) for use with other applications. 

* [DynamoRio repository]()
* [Spatter G/S Benchmark suite]()


### Download Tools and Repos

Download latest version build of Dynamorio (Dynamorio-Linux). We used a nightly build.
https://github.com/DynamoRIO/dynamorio/releases/tag/cronbuild-8.0.18895 

``` 
$ wget https://github.com/DynamoRIO/dynamorio/releases/download/cronbuild-8.0.18895/DynamoRIO-Linux-8.0.18895.tar.gz
$ tar –xvf DynamoRIO-Linux-8.0.18895.tar.gz 
# Download Spatter repo
$ git clone https://github.com/hpcgarage/spatter.git
```


### Edit DynamoRio cmake config files

```
git clone https://github.com/hpcgarage/trace_analysis.git
patch -u -b DynamoRIO-Linux-8.0.18895/cmake/DynamoRIOConfig.cmake -i trace_analysis/dynamorio_cmake.patch
```

#### Manual editing of files (only if not using patch)
```
$ vim DynamoRIO-Linux-8.0.18895/cmake/DynamoRIOConfig.cmake 
```

On line 533, comment out the lines which say
```
_DR_identify_clang() 
#if (NOT CMAKE_COMPILER_IS_GNUCC) 
#  # Our linker script is GNU-specific 
#  message(FATAL_ERROR "DynamoRIO's CMake configuration only supports the GNU linker on Linux") 
#endif (NOT CMAKE_COMPILER_IS_GNUCC) 
```

On line 1106, change that line that says 
`target_link_libraries(${target} dynamorio)`
to 
`target_link_libraries(${target} LINK_PUBLIC dynamorio)` 

One line 1490, update the line that says
```
if (NOT DynamoRIO_EXT_${extname}_NOLIB)
          target_link_libraries(${target} ${extname})
endif (NOT DynamoRIO_EXT_${extname}_NOLIB)
```
to 
`target_link_libraries(${target} LINK_PUBLIC ${extname})`

### Check out and build Spatter
We link Spatter with DynamoRio using the modified version of the toolchain from above:

```
$ cd spatter/ 
# Check out the DR branch
$ git checkout dynamorio_integration 
# Set path to Intel compilers
$ . /tools/misc/env/spatter_build.sh 

#export DYNAMORIO_ROOT=<directory_where_DR_is_installed>
export DYNAMORIO_ROOT=~/DynamoRIO-Linux-8.0.18895/
```

Configure and build Spatter
```
$ ./configure/configure_intel_dynamorio
$ cd ./build_intel_dynamorio
$ make -j
$ cd ../../
$ ./DynamoRIO-Linux-8.0.18895/bin64/drrun -noinject -c ./DynamoRIO-Linux-8.0.18895/samples/bin64/libopcodes.so -- ./spatter/build_intel_dynamorio/spatter -pUNIFORM:8:1 -t1
```

### **Trace register states**

```
$ git clone https://github.com/hpcgarage/trace_analysis.git
$ cd trace_analysis
$ git checkout dynamorio_client
$ cd client/
$ mkdir build
$ cd build/
$ cmake -DDynamoRIO_DIR=[dynamorio_path]/cmake/ ..
$ make
$ ./DynamoRIO-Linux-8.0.18895/bin64/drrun -noinject -c ./trace_analysis/client/build/libcount_client.so -- ./spatter/build_omp_intel/spatter -t1 -pUNIFORM:8:1
```

Histogram Instructions
```
$ touch out.log
$ ./DynamoRIO-Linux-8.0.18895/bin64/drrun -noinject -c ./trace_analysis/client/build/libcount_client.so -- ./spatter/build_omp_intel/spatter -t1 -pUNIFORM:8:1 &> out.log
$ python3 ./trace_analysis/histogram/generate_histogram.py out.log
```

