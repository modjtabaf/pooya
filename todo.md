
* use bazel
- parallel processing
* record processing order and reuse it
* folder structure
* use clang instead of gcc
- logging (spdlog)
- use gtest and cc_test
* make parents mandatory
* retire make files
- selective history
- yaml model definition
* scalar and array signals
- replace init virtual method with a template
* solver.cpp cleanup
- Doxygen
- A (virtual ?) method for verifying the number and types of input and output signals of a block
* internal arrays of ode solvers are defined as static. convert them to class members.
* change SignalId from integer to SignalInfo*
- rename signal() to create_signal()
