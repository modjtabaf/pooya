load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "gain_block",
    srcs = ["gain_block.cpp"],
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
    name = "memory_block",
    srcs = ["memory_block.cpp"],
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
    name = "delay_block",
    srcs = ["delay_block.cpp"],
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
    srcs = ["test03_integrator.cpp"],
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
    srcs = ["test04_mass_spring.cpp"],
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
    srcs = ["test05_pendulum.cpp"],
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
    srcs = ["test06_pendulum_with_torque.cpp"],
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
    srcs = ["test07_pendulum_with_pi.cpp"],
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
    srcs = ["test08_pendulum_with_pid.cpp"],
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
    srcs = ["test09_mass_spring_damper.cpp"],
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
    srcs = ["test10_memory_with_bus.cpp"],
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
    srcs = ["test11_int_gain.cpp"],
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
    name = "strsys",
    srcs = ["Steering_System.cpp"],
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
        "//tests/data:data",
    ],
)

cc_binary(
    name = "stm_old",
    srcs = ["simple_truck_model_oldpt1.cpp"],
    copts = [
        "-pedantic-errors",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-O3",
        "-fno-math-errno",
        "-ffast-math",
        ],
    defines = [
        "NDEBUG",
        ],
    linkopts = [
        "-lboost_iostreams",
        "-lboost_system",
        "-lboost_filesystem",
        ],
    deps = [
        "//src:core",
        "//src:misc",
        "//tests/data:data",
    ],
)

cc_binary(
    name = "stm_new",
    srcs = ["simple_truck_model_newpt1.cpp"],
    copts = [
        "-pedantic-errors",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-O3",
        "-fno-math-errno",
        "-ffast-math",
        ],
    defines = [
        "NDEBUG",
        ],
    linkopts = [
        "-lboost_iostreams",
        "-lboost_system",
        "-lboost_filesystem",
        ],
    deps = [
        "//src:core",
        "//src:misc",
        "//tests/data:data",
    ],
)
