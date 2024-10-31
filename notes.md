
* Where should values_array.h be? It was not needed and was removed.
- Benefits of using signal wrapper classes:
  - null signals may be avoided
  - operator= and casting operators may be defined

Roadmap:
* Always use smart pointers for signals and enforce it
* Decentralize signal values
* Discard _is_deriv if possible
* Decentralize signals
* discard Model
- use LabelSignal in bus
- discard BusSpec
- discard Block::init and combine it with the constructor
* Define and use signal wrappers
- make it possible to modify the model between major steps given init is called after modification
