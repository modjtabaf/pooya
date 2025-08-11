
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
* use gtest and cc_test
* Deprecate Model
* Merge Submodel into Submodel
* Named object
* Bus wrapper
* Fix <bool> specialization
* Discard id()
* Deprecate BusSpec
* In BusInfo, use both std::map and std::vector for signals so they can be accessed with either label or index

- Make sure Simulator evaluates the model only if necessary. Remove any duplicates.
- Call shared_from_this() only when necessary
- Convert current samples to tests (hybrid)
- Define pooya_assert_* and pooya_verify_* where pooya_assert_* macros are debug-only and a preprocessor directive is used to exclude pooya_verify_* macros from the release build
- Use static_assert() in pooya_assert_* macros
- Create an abstraction layer between pooya and Eigen
- Reduce the usage of POOYA_USE_SMART_PTRS through defining proper macros or helper classes
- Call pre_step and post_step for minor steps too
- enclose the methods that only contain pooya_verify macros within #if...#endif blocks
- Use DOT to generate a graph presentation of the model
- Doxygen
- logging (spdlog)
- parallel processing
- Hide SignalImpl shared pointers from the user and use them only when necessary
- Discard addsub and muldiv

<!-- - yaml model definition -->
<!-- - replace init virtual method with a template -->
<!-- - A (virtual ?) method for verifying the number and types of input and output signals of a block -->
<!-- - light weight Signal wrapper so it supports operator[] -->
<!-- - Make _assigned a debug-only flag (No. It is essential.) -->
<!-- - support auto state variables (not necessary, use pooya::Integrator instead) -->
<!-- - Make model mandatory -->
<!-- - Unify given_name and name of signals -->
<!-- - Remove Block::_parent -->
<!-- - move BusBlockBuilder::traverse_bus to BusSpec -->
