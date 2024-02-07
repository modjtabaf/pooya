
* use bazel
* record processing order and reuse it
* folder structure
* use clang instead of gcc
* make parents mandatory
* retire make files
* scalar and array signals
* solver.cpp cleanup
* internal arrays of ode solvers are defined as static. convert them to class members.
* change SignalId from integer to SignalInfo*
* rework Model and integrate SignalRegsitry into it
* introduce create_signal()
* introduce integer signals
* Deprecate the initial values for states
* template classes for single-input and single-output blocks

- enclose the methods that only contain verify macros within #if...#endif blocks
- Make _assigned a debug-only flag
- Use DOT to generate a graph presentation of the model
- Doxygen
- logging (spdlog)
- use gtest and cc_test
- selective history
- parallel processing
- move BusBlockBuilder::traverse_bus to BusSpec
- pass std::string as const ref
- In BusSignalInfo, use both std::map and std::vector for signals so they can be accessed with either label or index
- 
<!-- - yaml model definition -->
<!-- - replace init virtual method with a template -->
<!-- - A (virtual ?) method for verifying the number and types of input and output signals of a block -->
<!-- - light weight Signal wrapper so it supports operator[] -->
