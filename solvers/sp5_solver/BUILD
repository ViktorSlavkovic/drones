package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sp5_solver",
    srcs = ["sp5_solver.cc"],
    hdrs = ["sp5_solver.h"],
    linkstatic = True,
    deps = [
        "//:problem_cc_proto",
        "//:solution_cc_proto",
        "//solvers:problem_solver",
        "//solvers/ilp_solver:ilp_solver_cc_proto",
        "//solvers/util:lp_util",
        "@com_github_glog_glog//:glog",
        "@com_google_absl//absl/strings",
        "@third_party_ortools//ortools/base",
        "@third_party_ortools//ortools/linear_solver",
        "@third_party_ortools//ortools/linear_solver:linear_solver_cc_proto",
    ],
)
