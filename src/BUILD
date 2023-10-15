load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "core",
    srcs = glob(["core/*.cpp"]),
    hdrs = glob(["core/*.hpp"]),
    deps = [
        "//3rdparty:eigen",
        "//3rdparty:misc",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "misc",
    hdrs = glob(["misc/*.hpp"]),
    deps = [
        "//3rdparty:misc",
    ],
    visibility = ["//visibility:public"],
)
