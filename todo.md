
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
* selective history
* define pooya::ArrayN<N> as an alias to public Eigen::Array<double, N, 1>
* Create FloatSignalInfo
* pass std::string as const ref

- Create an abstraction layer between pooya and Eigen
- Reduce the usage of POOYA_USE_SMART_PTRS through defining proper macros or helper classes
- Call pre_step and post_step for minor steps too
- Make model mandatory
- enclose the methods that only contain pooya_verify macros within #if...#endif blocks
- Use DOT to generate a graph presentation of the model
- Doxygen
- logging (spdlog)
- use gtest and cc_test
- parallel processing
- move BusBlockBuilder::traverse_bus to BusSpec
- In BusSignalInfo, use both std::map and std::vector for signals so they can be accessed with either label or index

<!-- - yaml model definition -->
<!-- - replace init virtual method with a template -->
<!-- - A (virtual ?) method for verifying the number and types of input and output signals of a block -->
<!-- - light weight Signal wrapper so it supports operator[] -->
<!-- - Make _assigned a debug-only flag (No. It is essential.) -->
<!-- - support auto state variables (not necessary, use pooya::Integrator instead) -->
