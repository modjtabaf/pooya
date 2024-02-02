load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "test00",
    srcs = ["tests/test00_gain.cpp"],
    copts = [
        "-pedantic-errors",
        "-Wall",
        "-Wextra",
        "-Werror",
        ],
    linkopts = [
        "-lboost_iostreams",
        "-lboost_system",
        "-lboost_filesystem",
        ],
    deps = [
        "//src:core",
        "//src:misc",
    ],
)

cc_binary(
    name = "test01",
    srcs = ["tests/test01_memory.cpp"],
    copts = [
        "-pedantic-errors",
        "-Wall",
        "-Wextra",
        "-Werror",
        ],
    linkopts = [
        "-lboost_iostreams",
        "-lboost_system",
        "-lboost_filesystem",
        ],
    deps = [
        "//src:core",
        "//src:misc",
    ],
)

cc_binary(
    name = "test02",
    srcs = ["tests/test02_delay.cpp"],
    copts = [
        "-pedantic-errors",
        "-Wall",
        "-Wextra",
        "-Werror",
        ],
    linkopts = [
        "-lboost_iostreams",
        "-lboost_system",
        "-lboost_filesystem",
        ],
    deps = [
        "//src:core",
        "//src:misc",
    ],
)

cc_binary(
    name = "test03",
    srcs = ["tests/test03_integrator.cpp"],
    copts = [
        "-pedantic-errors",
        "-Wall",
        "-Wextra",
        "-Werror",
        ],
    linkopts = [
        "-lboost_iostreams",
        "-lboost_system",
        "-lboost_filesystem",
        ],
    deps = [
        "//src:core",
        "//src:misc",
    ],
)

cc_binary(
    name = "test04",
    srcs = ["tests/test04_mass_spring.cpp"],
    copts = [
        "-pedantic-errors",
        "-Wall",
        "-Wextra",
        "-Werror",
        ],
    linkopts = [
        "-lboost_iostreams",
        "-lboost_system",
        "-lboost_filesystem",
        ],
    deps = [
        "//src:core",
        "//src:misc",
    ],
)

cc_binary(
    name = "test05",
    srcs = ["tests/test05_pendulum.cpp"],
    copts = [
        "-pedantic-errors",
        "-Wall",
        "-Wextra",
        "-Werror",
        ],
    linkopts = [
        "-lboost_iostreams",
        "-lboost_system",
        "-lboost_filesystem",
        ],
    deps = [
        "//src:core",
        "//src:misc",
    ],
)

cc_binary(
    name = "test06",
    srcs = ["tests/test06_pendulum_with_torque.cpp"],
    copts = [
        "-pedantic-errors",
        "-Wall",
        "-Wextra",
        "-Werror",
        ],
    linkopts = [
        "-lboost_iostreams",
        "-lboost_system",
        "-lboost_filesystem",
        ],
    deps = [
        "//src:core",
        "//src:misc",
    ],
)

cc_binary(
    name = "test07",
    srcs = ["tests/test07_pendulum_with_pi.cpp"],
    copts = [
        "-pedantic-errors",
        "-Wall",
        "-Wextra",
        "-Werror",
        ],
    linkopts = [
        "-lboost_iostreams",
        "-lboost_system",
        "-lboost_filesystem",
        ],
    deps = [
        "//src:core",
        "//src:misc",
    ],
)

cc_binary(
    name = "test08",
    srcs = ["tests/test08_pendulum_with_pid.cpp"],
    copts = [
        "-pedantic-errors",
        "-Wall",
        "-Wextra",
        "-Werror",
        ],
    linkopts = [
        "-lboost_iostreams",
        "-lboost_system",
        "-lboost_filesystem",
        ],
    deps = [
        "//src:core",
        "//src:misc",
    ],
)

cc_binary(
    name = "test09",
    srcs = ["tests/test09_mass_spring_damper.cpp"],
    copts = [
        "-pedantic-errors",
        "-Wall",
        "-Wextra",
        "-Werror",
        ],
    linkopts = [
        "-lboost_iostreams",
        "-lboost_system",
        "-lboost_filesystem",
        ],
    deps = [
        "//src:core",
        "//src:misc",
    ],
)

cc_binary(
    name = "test10",
    srcs = ["tests/test10_memory_with_bus.cpp"],
    copts = [
        "-pedantic-errors",
        "-Wall",
        "-Wextra",
        "-Werror",
        ],
    linkopts = [
        "-lboost_iostreams",
        "-lboost_system",
        "-lboost_filesystem",
        ],
    deps = [
        "//src:core",
        "//src:misc",
    ],
)

cc_binary(
    name = "test11",
    srcs = ["tests/test11_int_gain.cpp"],
    copts = [
        "-pedantic-errors",
        "-Wall",
        "-Wextra",
        "-Werror",
        ],
    linkopts = [
        "-lboost_iostreams",
        "-lboost_system",
        "-lboost_filesystem",
        ],
    deps = [
        "//src:core",
        "//src:misc",
    ],
)

cc_test(
    name = "test_gain",
    size = "small",
    srcs = ["tests/test_gain.cpp"],
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
