
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

- Use DOT to generate a graph presentation of the model
- Doxygen
- logging (spdlog)
- use gtest and cc_test
- selective history
- parallel processing

<!-- - yaml model definition -->
<!-- - replace init virtual method with a template -->
<!-- - A (virtual ?) method for verifying the number and types of input and output signals of a block -->
<!-- - rename signal() to create_signal() -->
<!-- - light weight Signal wrapper so it supports operator[] -->
