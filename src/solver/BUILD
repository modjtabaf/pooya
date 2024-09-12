load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "solver",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.hpp"]),
    defines = [
        # "POOYA_USE_SMART_PTRS",
    ],
    deps = [
        # "@eigen",
        # "//3rdparty:misc",
        "//src/signal",
        "//src/block",
    ],
    visibility = ["//visibility:public"],
)
