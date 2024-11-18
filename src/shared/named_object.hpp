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

#ifndef __POOYA_NAMED_OBJECT_HPP__
#define __POOYA_NAMED_OBJECT_HPP__

#include <string>

namespace pooya
{

class ValidName
{
protected:
    std::string _name;

public:
    ValidName() = default;
    ValidName(const char* name) : _name(make_valid_name(name)) {}
    ValidName(const std::string& name) : _name(make_valid_name(name)) {}
    ValidName(const ValidName& root) = default;

    const std::string& str() const {return _name;}
    ValidName operator+(const std::string& name) const
    {
        ValidName ret;
        ret._name = _name + "/" + make_valid_name(name);
        return ret;
    }

    static std::string make_valid_name(const std::string& name);
};

class NamedObject
{
protected:
    ValidName _name;

public:
    NamedObject(const ValidName& name) : _name(name) {}

    const ValidName& name() const {return _name;}
}; // class NamedObject

}

#endif // __POOYA_NAMED_OBJECT_HPP__
