package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sp2_solver",
    srcs = ["sp2_solver.cc"],
    hdrs = ["sp2_solver.h"],
    linkstatic = True,
    deps = [
        "//:problem_cc_proto",
        "//solvers:problem_solver",
        "//:solution_cc_proto",
        "@com_github_glog_glog//:glog",
    ],
)

