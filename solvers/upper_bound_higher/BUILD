package(default_visibility = ["//visibility:public"])

cc_library(
    name = "upper_bound_higher",
    srcs = ["upper_bound_higher.cc"],
    hdrs = ["upper_bound_higher.h"],
    linkstatic = True,
    deps = [
        "//:problem_cc_proto",
        "//solvers/util:load_splitter",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_glog_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)
