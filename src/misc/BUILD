load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "misc",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.hpp"]),
    defines = [
        # "POOYA_USE_SMART_PTRS",
    ],
    deps = [
        "//3rdparty:misc",
    ],
    visibility = ["//visibility:public"],
)
