load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "test_gain",
    size = "small",
    srcs = ["test_gain.cpp"],
    deps = [
        "@com_google_googletest//:gtest_main",
        "//src:core",
        "//src:misc",
    ],
    linkopts = [
        "-lboost_iostreams",
        "-lboost_system",
        "-lboost_filesystem",
        ],
)

cc_test(
    name = "test_memory",
    size = "small",
    srcs = ["test_memory.cpp"],
    deps = [
        "@com_google_googletest//:gtest_main",
        "//src:core",
        "//src:misc",
    ],
    linkopts = [
        "-lboost_iostreams",
        "-lboost_system",
        "-lboost_filesystem",
        ],
)

cc_test(
    name = "test_delay",
    size = "small",
    srcs = ["test_delay.cpp"],
    deps = [
        "@com_google_googletest//:gtest_main",
        "//src:core",
        "//src:misc",
    ],
    linkopts = [
        "-lboost_iostreams",
        "-lboost_system",
        "-lboost_filesystem",
        ],
)
