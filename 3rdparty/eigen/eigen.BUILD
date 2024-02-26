load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "eigen",
    hdrs = glob(
        ["Eigen/**"],
        exclude = [
            "Eigen/src/OrderingMethods/Amd.h",
            "Eigen/src/SparseCholesky/**",
            "Eigen/Eigen",
            "Eigen/IterativeLinearSolvers",
            "Eigen/MetisSupport",
            "Eigen/Sparse",
            "Eigen/SparseCholesky",
            "Eigen/SparseLU",
        ],
    ),
    defines = [
        "EIGEN_MPL_ONLY",
        "EIGEN_NO_DEBUG",
    ],
    includes = [
        "eigen",
        ],
    visibility = ["//visibility:public"],
)
