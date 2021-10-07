## Using DynamoRio to track AVX-512 G/S instructions
Dynamorio provides several [sample clients](https://dynamorio.org/API_samples.html) as a demonstration of its API;
several of these clients are particularly useful when aiming to track specific instructions.

The [opcode_count](https://github.com/DynamoRIO/dynamorio/blob/master/api/samples/opcode_count.cpp) sample client provides a count of a specified instruction.
By default, it tracks the OP_add instruction.
The list of available opcodes for tracking within x86 is provided [here](https://github.com/DynamoRIO/dynamorio/blob/master/core/ir/x86/opcode.h).

Another option is using the [opcodes](https://github.com/DynamoRIO/dynamorio/blob/master/api/samples/opcodes.c) client, which counts all encountered instructions. By default, only the top 15 instructions are displayed.
However, it should be noted that the dynamorio stack has a limited size, so showing a large number of instruction counts may lead to the message buffer overflowing the stack.

Presumably, the execution count of scatter/gather instructions is relatively low compared to other instructions within an application, so modifying the opcodes client to only track s/g instructions may be useful. The dynamorio API provides the [`instr_is_scatter`](https://github.com/DynamoRIO/dynamorio/blob/ed78e2cc187702e66445c9eb5f2e3df479347a49/core/ir/instr.h#L1436) and [`instr_is_gather`](https://github.com/DynamoRIO/dynamorio/blob/ed78e2cc187702e66445c9eb5f2e3df479347a49/core/ir/instr.h#L1443) functions, so we can simply add a check before updating the drx counter.

While the [dynamorio release](https://dynamorio.org/page_releases.html) does come with these sample clients prebuilt for convenience's sake,
it may be more beneficial to build both Dynamorio and the sample clients yourself, in order to 1) modify the clients to better fit your needs and change the default options, as described above, and 2) avoid possible segfault/file not found bugs within the pre-built clients which may be caused by the client searching for libraries using the path provided when it was built.
Further details regarding building dynamorio can be found [here](https://dynamorio.org/page_building.html) and details regarding clients can be found [here](https://dynamorio.org/dynamorio_docs/using.html#sec_build).
It should also be noted that the sample clients should be compiled with the `SHOW_RESULTS` cflag in order for the results to be printed to console.

## Using the Dynamorio API to delimit trace regions
It may be useful to limit the code region executed under dynamorio to a specific section of code.
Dynamorio provides several options for users to achieve this.

The current recommended solution is to utilize the dynamorio [start/stop API](https://dynamorio.org/dynamorio_docs/API_BT.html#sec_startstop).
This requires linking dynamorio to your target application, details of which are located [here](https://dynamorio.org/dynamorio_docs/using.html#sec_build).
However, instead of using `configure_DynamoRIO_client`, since we are linking dynamorio to our target app,
we want to use `configure_DynamoRIO_standalone`.
See [Dynamorio's cmake config file](https://github.com/DynamoRIO/dynamorio/blob/master/make/DynamoRIOConfig.cmake.in)
for more details.
Before calling the standalone configuration helper function, we will also want to specify our `RUNTIME_OUTPUT_DIRECTORY` to be
the directory of where our target application will be built.

### Example cmake

```
add_definitions(-DUNIX)
add_definitions(-DX86_64)

find_package(DynamoRIO)
if (NOT DynamoRIO_FOUND)
  message(FATAL_ERROR "DynamoRIO package required to build")
endif ()

set_target_properties(spatter PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${BUILD_DIR}")

configure_DynamoRIO_standalone(spatter)
```

Before building your target application (now with dynamorio linked), we will also want to actually specify which region(s) of code we want dynamorio
to run on. To do this, simply include the api header file with `#include "dr_api.h"` and use the `dr_app_setup_and_start` and `dr_app_stop_and_cleanup` methods to delimit the desired trace region. Alternatively, you can call the setup, start, stop, and cleanup methods separately if there are multiple desired code regions to trace.

Once the target application has been built, we can run dynamorio with
```
./drrun -noinject -c [path/to/desired_client] -- [path/to/target_app] [target app args]
```
Note that `-noinject` is essential because drrun would otherwise inject dynamorio into the target app, which is unnecessary since we have already linked our app with dynamorio. Not including this option will most likely cause a segfault where dynamorio "`Cannot correctly handle signal 11 in thread`".

### Example run command:
```
~/dynamorio/build/bin64/drrun -noinject -c ~/dynamorio/build/api/bin/libmemtrace_x86_binary.so -- ~/spatter/build_omp_gnu/spatter -pUNIFORM:8:1
```

### Additional Notes:
- Use the most recent build instead of the 8.0.0-1 release, as there is a bug (fixed in [this](https://github.com/DynamoRIO/dynamorio/commit/418c10ce6982b08d043184aa2969ad795e323fc5) commit)
where attempting to use the `configure_DynamoRIO_standalone` helper command causes dynamorio to complain about not being able to find the library output directory. Further details regarding building dynamorio can be found [here](https://dynamorio.org/page_building.html).
- In order to include the sample clients when building dynamorio, use the `BUILD_SAMPLES` cmake flag.\
Example:
```
$ git clone https://github.com/DynamoRIO/dynamorio.git
$ cd dynamorio && mkdir build && cd build
$ cmake -DBUILD_SAMPLES=ON ..
$ make -j
```
- Spatter uses `LINK_PUBLIC` to link libraries while dynamorio does not.
Since all uses of target_link_libraries with a target must be either all-keyword or all-plain, this will cause cmake to refuse to build.
In order to fix this, change line 1097 of `dynamorio/build/cmake/DynamoRIOConfig.cmake` to use `LINK_PUBLIC`.
If building a client and using extensions, then line 1481 of `dynamorio/build/cmake/DynamoRIOConfig.cmake`
will also need to be changed to use `LINK_PUBLIC`.
- As of March 2021, dynamorio does not support compiling and linking target apps compiled with icc.
However, it should be safe to simply disable the error message found on line 261 of `dynamorio/make/DynamoRIOConfig.cmake.in`,
and compiling should proceed smoothly.  
https://groups.google.com/g/dynamorio-users/c/w-VRqWqym28/m/RfqYNYdCAQAJ  
https://github.com/DynamoRIO/dynamorio/issues/4794  
- The dynamorio_intergration branch for spatter serves as an example for delimiting dynamorio traces.

A much simpler albeit much less flexible approach would be to use drrun `-trace_after_instrs` and `-exit_after_tracing` command line arguments to delimit trace regions. However, this requires knowledge of the exact instruction number to start/stop at, and cannot be used to trace multiple regions.

Another possible solution in the future is to utilize the (as of March 2021, not yet implemented) dynamorio trace delimiting annotations.
As Derek Bruening (dynamorio developer) notes, annotations would be able to "be compiled into the binary with minimal changes as opposed to linking with drcachesim and DR, and they are finer-grained and could be used to do things while tracing like turn on and off online cache filtering or add labels into the trace demarking regions of interest".
Therefore it may be worth tracking when the delimiting annotations are implemented, as they may be superior to the current stop/start API.\
The following issues are related to the delimiting annotations implementation:\
https://github.com/DynamoRIO/dynamorio/issues/2478 \
https://github.com/DynamoRIO/dynamorio/issues/3107 \
https://github.com/DynamoRIO/dynamorio/issues/3995


## Disassembly
It can be helpful to disassemble your target application to double-check that scatter/gather instructions actually exist within the binary.
We have several helpful tools at our disposal to achieve this.

The simplest is simply using objdump, which will give the instructions found in your app: 
`objdump -d [app]`

Dynamorio also can be used for disassembly via the static library drdecode. Details can be found [here](https://dynamorio.org/dynamorio_docs/page_standalone.html).

GDB is also an option for [disassembly](https://visualgdb.com/gdbreference/commands/disassemble), with the added benefit of being able to set breakpoints on specific instructions. Note that you will have to dereference the address when setting the breakpoint (Eg, `b *0x1234`).\
Spatter, when compiled with optimization level 3, contains scatter/gather instructions, but is prone to
skipping over said instructions, which was discovered with gdb.
Compiling with less aggressive optimization level 2 instead fixed this problem.

## Common Gather/Scatter Instructions for x86 and Arm
For the most common platforms we work with, x86 and Arm, you would expect to see the following vectorized gather/scatter instructions:

floating point scatters:  
VSCATTERDPD, VSCATTERDPS, VSCATTERQPD, VSCATTERQPS  

scatters:  
VPSCATTERDD, VPSCATTERDQ, VPSCATTERQD, VPSCATTERQQ  

floating point gathers:  
VGATHERDPD, VGATHERDPS, VGATHERQPD, VGATHERQPS  

gathers:  
VPGATHERDD, VPGATHERDQ, VPGATHERQD, VPGATHERQQ  

Note that dynamorio does not support sparse gathers.  

* [Intel AVX Intrinsics Guide](https://software.intel.com/sites/landingpage/IntrinsicsGuide/#text=vscatter&avx512techs=AVX512F)
* [SVE document]
* [Intel 64 and IA-32 Architectures Software Developer’s Manual Volume 2 (2A, 2B, 2C & 2D): Instruction Set Reference, A-Z](https://www.intel.com/content/dam/www/public/us/en/documents/manuals/64-ia-32-architectures-software-developer-instruction-set-reference-manual-325383.pdf)  
Pages 5-251 through 5-285 for Gather instructions  
Pages 5-544 through 5-553 for Scatter instructions  

## Validation Of Spatter Gather or Scatter functions
With the tools outlined here we can do the following: 
1) Generate code with vectorized scatter/gather instructions (ie, AVX-512 or SVE)
2) Disassemble and run our app with gdb to observe the execution of G/S instructions.
3) Run DynamoRio with the Gather/Scatter function to capture the number of G/S instructions within a specific function and compare with the static analysis of the code. The number may be not exact due to offsets but it should be within the same order of magnitude. 

As an example we compile spatter with the intel compiler (icc) with the following flags:
Note we compile with O2 optimization level to reduce the chance of aggressive code optimization.
```
FLAGS="-O2 -xCOMMON-AVX512 -g"
```
Using gdb, we disassemble the desired function to set and ignore breakpoints on S/G instructions in order to count the number of executions of said S/G instructions.

```
$ gdb ./spatter
(gdb) disassemble gather_smallbuf
[output truncated to show only gather instructions]
    0x0000000000416a0b <+389>:   vgatherqpd (%r15,%zmm2,8),%zmm3{%k1}
    0x0000000000416a4f <+457>:   vgatherqpd (%r15,%zmm3,8),%zmm4{%k1}
(gdb) b *0x0000000000416a0b
Breakpoint 1 at 0x416a0b: file /nethome/vhuang31/spatter/src/openmp/openmp_kernels.c, line 44.
(gdb) b *0x0000000000416a4f
Breakpoint 2 at 0x416a4f: file /nethome/vhuang31/spatter/src/openmp/openmp_kernels.c, line 44.
(gdb) ignore 1 999999999
Will ignore next 999999999 crossings of breakpoint 1.
(gdb) ignore 2 999999999
Will ignore next 999999999 crossings of breakpoint 2.
(gdb) r -pUNIFORM:8:1 -l$((2**16)) -t1 -kGather
[output omitted]
(gdb) info breakpoints
Num     Type           Disp Enb Address            What
1       breakpoint     keep y   0x0000000000416a0b in L_gather_smallbuf_20__par_region0_2_0 at /nethome/vhuang31/spatter/src/openmp/openmp_kernels.c:44
        breakpoint already hit 720896 times
        ignore next 995183746 hits
2       breakpoint     keep y   0x0000000000416a4f in L_gather_smallbuf_20__par_region0_2_0 at /nethome/vhuang31/spatter/src/openmp/openmp_kernels.c:44
        ignore next 999999999 hits
```

and compare to output from DR with the modified opcodes client on the spatter target application with the dynamorio trace delimited
to the gather kernel via the start/stop API for this function
```
$ ~/dynamorio/build/bin64/drrun -noinject -c ~/trace_analysis/client/build/libcount_client.so -- ~/spatter/build_omp_intel/spatter -pUNIFORM:8:1 -l$((2**16)) -t1 -kGather
[output truncated]
Top 15 opcode execution counts in 64-bit AMD64 mode:
     720896 : vgatherqpd
```

Note that the vpgatherdq instructions occur elsewhere in the app, which can be eliminated via the trace limiting techniques discussed above.
We can see that both the dynamorio client and gdb report the same number of executed `vgatherqpd`.

Furthermore, we can examine the spatter source code to determine that by default,
Spatter does 11 runs of the specified length (10 runs + 1 cache warmup run),
so our expected number of gathers is 2^16 * 11 = 720896,
which indeed matches up with the numbers dynamorio and gdb report.


### Why use dynamorio over gdb?
Gdb has several disadvantages which are worth considering
1) Gdb requires manual identification, breakpoint setting, and breakpoint ignoring of individual S/G instructions
2) Gdb has a maximum of 2^31 - 1 on the number of times it will ignore a breakpoint
3) Gdb's execution time is significantly longer than dynamorio's; (Running spatter with a length of 2^16 takes gdb roughly a minute, while the dynamorio client takes less than a second. This difference is even more pronounced with larger lengths.) 



## Dynamorio Resources
Check out these resources to learn more about Dynamorio!

[Dynamorio API](https://dynamorio.org/dynamorio_docs/index.html)\
[Dynamorio Repo](https://github.com/DynamoRIO/dynamorio)\
[Dynamorio Help Forums](https://groups.google.com/g/dynamorio-users)\
Additional detail can also be found within the source code. For example, additional cmake deployment details are located in [Dynamorio's cmake config file](https://github.com/DynamoRIO/dynamorio/blob/master/make/DynamoRIOConfig.cmake.in).
