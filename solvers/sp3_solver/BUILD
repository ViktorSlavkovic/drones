package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sp3_solver",
    srcs = ["sp3_solver.cc"],
    hdrs = ["sp3_solver.h"],
    linkstatic = True,
    deps = [
        "//:problem_cc_proto",
        "//solvers:problem_solver",
        "//:solution_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_glog_glog//:glog",
        "@third_party_ortools//ortools/constraint_solver:routing",
        "@third_party_ortools//ortools/constraint_solver:routing_enums_cc_proto",
        "@third_party_ortools//ortools/constraint_solver:routing_index_manager",
        "@third_party_ortools//ortools/constraint_solver:routing_parameters",
    ],
)
