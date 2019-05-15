package(default_visibility = ["//visibility:public"])

cc_library(
    name = "upper_bound",
    srcs = ["upper_bound.cc"],
    hdrs = ["upper_bound.h"],
    linkstatic = True,
    deps = [
        "//:problem_cc_proto",
        "//solvers/util:load_splitter",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_glog_glog//:glog",
        "@com_google_absl//absl/strings",
        "@third_party_ortools//ortools/constraint_solver:routing",
        "@third_party_ortools//ortools/constraint_solver:routing_enums_cc_proto",
        "@third_party_ortools//ortools/constraint_solver:routing_index_manager",
        "@third_party_ortools//ortools/constraint_solver:routing_parameters",
    ],
)
