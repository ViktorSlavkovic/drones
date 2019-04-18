package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sp1_solver",
    srcs = ["sp1_solver.cc"],
    hdrs = ["sp1_solver.h"],
    linkstatic = True,
    deps = [
        "//:problem_cc_proto",
        "//:solution_cc_proto",
        "//solvers:problem_solver",
        "@com_github_glog_glog//:glog",
    ],
)
