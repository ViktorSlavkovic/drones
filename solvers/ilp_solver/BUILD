package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ilp_solver",
    srcs = ["ilp_solver.cc"],
    hdrs = ["ilp_solver.h"],
    linkstatic = True,
    deps = [
        ":ilp_solver_cc_proto",
        "//:problem_cc_proto",
        "//:solution_cc_proto",
        "//solvers:problem_solver",
        "//solvers/util:lp_util",
        "@com_github_glog_glog//:glog",
        "@com_google_absl//absl/strings",
        "@third_party_ortools//ortools/base",
        "@third_party_ortools//ortools/linear_solver",
        "@third_party_ortools//ortools/linear_solver:linear_solver_cc_proto",
    ],
)

cc_proto_library(
    name = "ilp_solver_cc_proto",
    deps = [":ilp_solver_proto"],
)

proto_library(
    name = "ilp_solver_proto",
    srcs = ["ilp_solver.proto"],
)
