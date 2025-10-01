# Copyright 2024 Mojtaba (Moji) Fathi

#  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”),
# to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

#  THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
# WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

SHARED_COPTS = []

def get_from_dict(dct, key, val=[]):
    if key in dct:
        val = dct[key]
        dct.pop(key)
    return val

def pooya_cc_library(name, srcs, **kwargs):
    hdrs = get_from_dict(kwargs, "hdrs")
    deps = get_from_dict(kwargs, "deps")

    native.cc_library(
        name = name,
        srcs = srcs,
        hdrs = hdrs,
        copts = [
            "-pedantic-errors",
            "-Wall",
            "-Wextra",
            "-Werror",
            ] + SHARED_COPTS,
        linkopts = [
            "-lboost_iostreams",
            "-lboost_system",
            "-lboost_filesystem",
            ],
        deps = deps,
        **kwargs
    )

def pooya_cc_binary(name, src, **kwargs):
    deps = get_from_dict(kwargs, "deps")

    native.cc_binary(
        name = name,
        srcs = [src],
        copts = [
            "-pedantic-errors",
            "-Wall",
            "-Wextra",
            "-Werror",
            ] + SHARED_COPTS,
        linkopts = [
            "-lboost_iostreams",
            "-lboost_system",
            "-lboost_filesystem",
            ],
        deps = [
            "//src/block",
            "//src/solver",
            "//src/misc",
        ] + deps,
        **kwargs
    )

def pooya_cc_test(name, src, **kwargs):
    deps = get_from_dict(kwargs, "deps")

    native.cc_test(
        name = name,
        size = "small",
        srcs = [src],
        copts = SHARED_COPTS,
        deps = [
            "@com_google_googletest//:gtest_main",
        ] + deps,
        linkopts = [
            "-lboost_iostreams",
            "-lboost_system",
            "-lboost_filesystem",
            ],
        **kwargs
    )
