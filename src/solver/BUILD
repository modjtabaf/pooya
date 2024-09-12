load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "solver",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.hpp"]),
    defines = [
        # "POOYA_USE_SMART_PTRS",
    ],
    deps = [
        "//src/signal",
        "//src/block",
    ],
    visibility = ["//visibility:public"],
)
