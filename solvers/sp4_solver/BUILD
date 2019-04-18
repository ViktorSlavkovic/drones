package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sp4_solver",
    srcs = ["sp4_solver.cc"],
    hdrs = ["sp4_solver.h"],
    linkstatic = True,
    deps = [
        "//:problem_cc_proto",
        "//:solution_cc_proto",
        "//solvers:problem_solver",
        "//solvers/util:allocator",
        "@com_github_glog_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)
