These instructions were tested on an Ubuntu 18.04 server with Cascade Lake CPU.

We use three different repositories to demonstrate how to build this trace tool for 1) validation with Spatter 
and 2) for use with other applications. 

* This repository
* [DynamoRio repository](https://github.com/DynamoRIO/dynamorio)  
**One of**
* [Spatter G/S Benchmark suite](https://github.com/hpcgarage/spatter)
* [LULESH (modified)](https://github.com/huangvincent170/LULESH)
* [AMG (modified)](https://github.com/huangvincent170/AMG)
* Your desired target app


## **Download Tools and Repos**

1) Clone this repository
```
working_dir $git clone https://github.com/hpcgarage/dr-gather-scatter-trace.git
```

2) Download latest version build of Dynamorio (Dynamorio-Linux). We used a nightly build.
https://github.com/DynamoRIO/dynamorio/releases/tag/cronbuild-8.0.18895 

``` 
$ wget https://github.com/DynamoRIO/dynamorio/releases/download/cronbuild-8.0.18895/DynamoRIO-Linux-8.0.18895.tar.gz
$ tar –xvf DynamoRIO-Linux-8.0.18895.tar.gz 
```

3a) Clone Spatter repo.
```
$ git clone https://github.com/hpcgarage/spatter.git
```

3b) Clone your target app repo.
Example target apps include `LULESH`, `AMG`, or `PENNANT`.

## **Edit DynamoRio cmake config files**

This patch file will
1) Enable compilation of target apps with non-gnucc compilers,
such as the intel compilers.
2) Modify the library linkage mode to `LINK_PUBLIC`. Spatter uses the `LINK_PUBLIC` mode for its libraries, and Cmake requires the same mode to be used for all libaries. If your target app does not use the `LINK_PUBLIC` mode, you may either switch to `LINK_PUBLIC`, or revert this change.

```
git clone https://github.com/hpcgarage/trace_analysis.git
patch -u -b DynamoRIO-Linux-8.0.18895/cmake/DynamoRIOConfig.cmake -i trace_analysis/dynamorio_cmake.patch
```


## **Build Spatter with DynamoRio Support**
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

## **Build Target App with Dynamorio Support**
We provide an example of linking Dynamorio with the `LULESH` app via [this commit](https://github.com/huangvincent170/LULESH/commit/987ac29db88556c98391302808b2dbb27123e296),
and the `PENNANT` app via [this commit](https://github.com/huangvincent170/PENNANT/commit/eb1a3c5b7f36a008e2d64020987e8b26fd18afdd).

`LULESH` uses a CMake build system, while `PENNANT` uses Make.

Note that these commits not only link Dynamorio with the target app,
they also modify the code's pragmas to generate G/S instructions.

View the Dynamorio [official documentation](https://dynamorio.org/page_build_client.html)
and the [in-code documentation](https://github.com/DynamoRIO/dynamorio/blob/624bc4aa3dd3c373018778fb96cc4af8aa5ea87e/make/DynamoRIOConfig.cmake.in#L32)
for additional details regarding linking Dynamorio.


## **Trace register states**

```
$ git clone https://github.com/hpcgarage/dr-gather-scatter-trace.git
$ cd dr-gather-scatter-trace
$ cd client/
$ mkdir build
$ cd build/
$ cmake -DDynamoRIO_DIR=[dynamorio_path]/cmake/ ..
$ make
$ ./DynamoRIO-Linux-8.0.18895/bin64/drrun -noinject -c ./trace_analysis/client/build/libcount_client.so -- [target_app_path] [target_app_args]
```

# **Deprecated Instructions**

**Note**: Histogram functionality is now built into the client  
Histogram Instructions
```
$ touch out.log
$ ./DynamoRIO-Linux-8.0.18895/bin64/drrun -noinject -c ./trace_analysis/client/build/libcount_client.so -- ./spatter/build_omp_intel/spatter -t1 -pUNIFORM:8:1 &> out.log
$ python3 ./trace_analysis/histogram/generate_histogram.py out.log
```

#### Manual editing of DynamoRio config files (optional)
**Note: This step is only needed if you are not using the patch above!**

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

