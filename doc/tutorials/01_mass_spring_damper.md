## Linear Mass-Spring-Damper

In this tutorial, you'll see how the unit step response of a simple mass-spring-damper system may be found with **pooya**. Follow the steps below and find out:

### Governing Equation
The governing equation of the system is

```math
m \ddot x + c \dot x + k x = F(t) \implies \ddot x = \frac{1}{m} F(t) - \frac{c}{m} \dot x - \frac{k}{m} x 
```

which may be represented as the following block diagram

![mass_spring_damper1 drawio](https://github.com/modjtabaf/pooya/assets/10777383/0349cd98-aa01-4bf7-ac8b-a680d2ad3bcf)

### Implement Using Pooya Blocks

**Pooya** comes equipped with a small set of commonly used blocks, e.g. gain, integrator, addition/subtraction, memory, etc.
The first step is to identify the blocks and their equivalent **pooya** types and label them. In the diagram below, the identified blocks are labeled followed by their **pooya** block types in green. The entire model is encapsulated as a *pooya::Model* object named *model*:

![mass_spring_damper2 drawio](https://github.com/modjtabaf/pooya/assets/10777383/d95ae10b-dcda-4431-9dc5-bd99860107e6)

Here is the implementation:

```cpp
// define model parameters
constexpr double m{0.25}, c{0.3}, k{0.7};

// create pooya blocks
pooya::Model      model("model");
pooya::Source     src_F("F(t)",
    [](double t) -> double
    {
        return t < 1 ? 0 : 1;
    });
pooya::Gain       gain_1_m("1_m", 1 / m);
pooya::AddSub     addsub_rhs("rhs", "+--");
pooya::Integrator int_xdd("xdd");
pooya::Integrator int_xd("xd");
pooya::Gain       gain_c_m("c_m", c / m);
pooya::Gain       gain_k_m("k_m", k / m);
```

The second step is to identify the signals and their equivalent **pooya** signal types and label them. The most common signal type is a scalar signal which represents a scalar real number. In this block diagram, all signals are scalars so it is not needed to mention the signal types. In the diagram below, the identified signals are prefixed *s_* and labeled in red:

![mass_spring_damper3 drawio](https://github.com/modjtabaf/pooya/assets/10777383/b96a2d40-6a90-44d3-b432-cc8c7e8f2978)

Here is the implementation:

```cpp
// create signals
pooya::ScalarSignalId s_F     = model.create_scalar_signal("F");
pooya::ScalarSignalId s_F_m   = model.create_scalar_signal("F_m");
pooya::ScalarSignalId s_xdd   = model.create_scalar_signal("xdd");
pooya::ScalarSignalId s_xd    = model.create_scalar_signal("xd");
pooya::ScalarSignalId s_x     = model.create_scalar_signal("x");
pooya::ScalarSignalId s_kx_m  = model.create_scalar_signal("kx_m");
pooya::ScalarSignalId s_cxd_m = model.create_scalar_signal("cxd_m");
```

Now that all blocks and signals are defined, the model could be set up by adding the blocks and defining input and output signals:

```cpp
// setup the model
model.add_block(src_F, {}, s_F);
model.add_block(gain_1_m, s_F, s_F_m);
model.add_block(addsub_rhs, {s_F_m, s_cxd_m, s_kx_m}, s_xdd);
model.add_block(int_xdd, s_xdd, s_xd);
model.add_block(int_xd, s_xd, s_x);
model.add_block(gain_c_m, s_xd, s_cxd_m);
model.add_block(gain_k_m, s_x, s_kx_m);
```

The model setup is complete and it is ready to use. The RK4 solver works well for this model. A simulator object is needed for simulating along with a history object for recording the signal values. While the history object is optional for simulation, it is useful for post-processing. Here, the simulation starts at `t0 = 0` and is run for 10 seconds with a fixed time step of 0.1 seconds:

```cpp
// set up the simulator
pooya::Rk4 solver;
pooya::Simulator sim(model, nullptr, &solver);
pooya::History history(model);

// run the simulation
double t;
for (uint k=0; pooya::arange(k, t, 0, 10, 0.1); k++)
{
    sim.run(t);
    history.update(k, t);
}
```

After the simulation is done, the history can be saved as a csv file for further analysis:

```cpp
// post-processing
history.shrink_to_fit();
history.export_csv("pooya_tutorial_01_mass_spring_damper.csv");
```

The complete source file is located at *samples/tut01_mass_spring_damper.cpp*. If you have **bazel** installed, you may execute this command in a terminal to build and run this sample:

```bash
bazel run //samples:tutorial01
```

If **gnuplot** is installed in your system, a plot of the simulation results will be shown:

![image](https://github.com/modjtabaf/pooya/assets/10777383/b7605e4f-f7f9-4219-b266-758e978e8622)

The first **pooya** tutorial is complete!
