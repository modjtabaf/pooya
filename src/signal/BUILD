load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "signal",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.hpp"]),
    defines = [
        # "POOYA_USE_SMART_PTRS",
    ],
    deps = [
        "@eigen",
        "//src/helper",
    ],
    visibility = ["//visibility:public"],
)
