# DynamoRio Gather/Scatter Trace Tool
This repository provides an example of how to use DynamoRio to generate gather/scatter patterns that can be used either with a simulator or with the [Spatter gather/scatter benchmark suite](https://github.com/hpcgarage/spatter/).

## Getting Started
Please read through the following documentation pages:
* [Building and testing this tool](https://github.com/hpcgarage/dr-gather-scatter-trace/blob/main/docs/building-gs-trace-tools.md)
* [Validation of G/S for real-world codes](https://github.com/hpcgarage/dr-gather-scatter-trace/blob/main/docs/building-gs-trace-tools.md)
* [Using this tool with PENNANT code sample](https://github.com/hpcgarage/dr-gather-scatter-trace/blob/main/docs/application-testing.md)

## Motivation for this Tool
The figure here shows our current state-of-the-art (top) and our desired tool workflow (bottom). We typically generate large memory traces to try and replicate useful memory behaviors from applications. With our current workflow, we might use a tool like DynamoRio or Pin to generate memory traces and then feed those into a simulator. In our desired workflow, we would like to parse memory traces in an online fashion to generate _representive sparse access patterns_. These patterns can then be used as inputs to architectural simulators (via tools like SST's Miranda trace generator) or sparse access benchmark suites like our Spatter benchmark.

<img src="https://github.com/hpcgarage/dr-gather-scatter-trace/blob/main/docs/presentations/figs/dr-trace-generator-overview.png" width="600" height="400" />

This tool attempts to provide the means to parse applications in a reasonable amount of time and then provide reusable patterns that can be used to investigate and validate performance for a specific computer architecture. 

### Related Work
* MODSIM21 [slides](https://github.com/hpcgarage/dr-gather-scatter-trace/blob/main/docs/presentations/young-etal-scatter-gather-analysis-slides-MODSIM-2021.pdf) and [poster](https://github.com/hpcgarage/dr-gather-scatter-trace/blob/main/docs/presentations/young-etal-gather-scatter-analysis-poster-MODSIM21.pdf) detailing this work
* [Spatter gather/scatter benchmark suite](https://github.com/hpcgarage/spatter)


## Acknowledgments
This work is sponsored by [NSF OAC-1710371](https://www.nsf.gov/awardsearch/showAward?AWD_ID=1710371&HistoricalAwards=false) and a contract with Sandia National Laboratories. 
