/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "src/helper/trace.hpp"
#include "src/helper/verify.hpp"
#include "src/helper/util.hpp"
#include "src/signal/signal_id.hpp"
#include "src/signal/float_signal.hpp"
#include "src/signal/scalar_signal.hpp"
#include "src/signal/array_signal.hpp"
#include "src/signal/int_signal.hpp"
#include "src/signal/bool_signal.hpp"
#include "model.hpp"

namespace pooya
{

// SignalId Model::lookup_signal(const std::string& name, bool exact_match) const
// {
//     pooya_trace("block: " + full_name());
//     if (name.empty()) {return SignalId();}

//     auto name_len = name.length();

//     auto it = std::find_if(_signal_infos.begin(), _signal_infos.end(),
//         [&] (SignalId sig) -> bool
//         {
//             if (exact_match) {return sig->_full_name == name;}
                
//             auto str_len = sig->_full_name.length();
//             return (str_len >= name_len) && (sig->_full_name.substr(str_len - name_len) == name);
//         });

//     return it == _signal_infos.end() ? SignalId() : *it;
// }

// void Model::register_state_variable(FloatSignalId sig, FloatSignalId deriv_sig)
// {
//     pooya_trace("block: " + full_name());
//     pooya_verify_valid_signal(sig);
//     pooya_verify_valid_signal(deriv_sig);
//     pooya_verify(!sig->is_state_variable(), sig->_full_name + ": signal is already registered as a state variable!");
//     pooya_verify((sig->is_scalar() && deriv_sig->is_scalar()) || (sig->is_array() && deriv_sig->is_array() && sig->as_array()._size == deriv_sig->as_array()._size),
//         sig->_full_name + ", " + deriv_sig->_full_name + ": type or size mismatch!");

//     _signal_infos[sig->_index]->as_float()._deriv_sig = deriv_sig;
// }

Model::Model(const std::string& given_name) : Parent(given_name, 0, 0)
{
    pooya_trace("block: " + full_name());
    _given_name = make_valid_given_name(_given_name);
    _full_name = "/" + _given_name;
    _initialized = true;
}

bool Model::init(Parent& /*parent*/, BusId /*ibus*/, BusId /*obus*/)
{
    return true;
}

// void Model::invalidate()
// {
//     for (auto& sig: _signal_infos)
//     {
//         if (sig->is_value())
//         {
//             sig->as_value().clear();
//         }
//     }
// }

}
