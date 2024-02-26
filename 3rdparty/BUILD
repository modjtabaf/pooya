load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "misc",
    hdrs = glob(["misc/**"]),
    deps = [
        "@eigen",
        ],
    includes = [
        ".",
        ],
    visibility = ["//visibility:public"],
)
