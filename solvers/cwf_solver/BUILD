package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cwf_solver",
    srcs = ["cwf_solver.cc"],
    hdrs = ["cwf_solver.h"],
    linkstatic = True,
    deps = [
        "//:problem_cc_proto",
        "//:solution_cc_proto",
        "//solvers:problem_solver",
        "//solvers/util:allocator",
        "//solvers/util:load_splitter",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_glog_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)
