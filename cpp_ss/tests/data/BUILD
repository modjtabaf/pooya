load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "data",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.hpp"]),
    deps = [
        "//3rdparty:eigen",
    ],
    visibility = ["//visibility:public"],
)
