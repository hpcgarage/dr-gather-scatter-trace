*Note:* This page is in progress as we are currently working on this test case for the tool. In short, this example works but it is quite slow, and we are currently looking at techniques to speed the analysis up. 

## Application tracing with the dr-gather-scatter-trace tool

If you want to scope the application using DynamoRio commands, you can add any of the following commands to your code and include the appropriate header file.

```
dr_app_setup_and_start();
dr_app_stop_and_cleanup();
OR 
dr_app_setup();
dr_app_start();
dr_app_stop();
dr_app_start();
dr_app_stop();
dr_app_cleanup();
```


## PENNANT Example
[PENNANT](https://github.com/lanl/PENNANT) is an unstructured mesh mini-app designed by Los Alamos National Lab based on the larger application, FLAG. 

Compile the application - we are using our local install of Intel OneAPI for compilation

```
#Source Intel tools
$ . /tools/intel/oneapi/2021.3/setvars.sh

#Include DynamoRio path to pull in flags and includes
$ export DYNAMORIO_ROOT=~/DynamoRIO-Linux-8.0.18895/
$ make -j
...
icpc -o build/pennant build/ExportGold.o build/Parallel.o build/WriteXY.o build/QCS.o build/TTS.o build/main.o build/Mesh.o build/InputFile.o build/HydroBC.o build/GenMesh.o build/Driver.o build/Hydro.o build/PolyGas.o -qopenmp -L~/DynamoRIO-Linux-8.0.18895/lib64/release/ -l dynamorio
```

Run the code with the DynamoRio tool. 
```
$ export LD_LIBRARY_PATH=${DYNAMORIO_ROOT}/lib64/release:$LD_LIBRARY_PATH

#   Remember to run with noinject since we are linking against libdynamorio to use trace delimiting!
#   Also, we currently suggest running just with one thread.
$ export OMP_NUM_THREADS=1
$ ./DynamoRIO-Linux-8.0.18895/bin64/drrun -noinject -c ./dr-gather-scatter-trace/client/build/libcount_client.so -- ./PENNANT/build/pennant ./PENNANT/test/sedovflat/sedovflat_1920.pnt &> pennant_sedovflat_1920.out
```

Run the histogram tool to pull out specific patterns (note this is the slow bit!)
```

```
