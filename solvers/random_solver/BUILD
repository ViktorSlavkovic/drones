package(default_visibility = ["//visibility:public"])

cc_library(
    name = "random_solver",
    srcs = ["random_solver.cc"],
    hdrs = ["random_solver.h"],
    linkstatic = True,
    deps = [
        "//:problem_cc_proto",
        "//:solution_cc_proto",
        "//solvers:problem_solver",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_glog_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)
