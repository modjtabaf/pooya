load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "block",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.hpp"]),
    defines = [
        # "POOYA_USE_SMART_PTRS",
    ],
    deps = [
        "//src/signal",
    ],
    visibility = ["//visibility:public"],
)
