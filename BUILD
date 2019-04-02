cc_binary(
    name = "sp2_main",
    srcs = ["sp2_main.cc"],
    linkstatic = True,
    deps = [
        ":problem_cc_proto",
        ":problem_manager",
        ":problem_solver_factory",
        ":solution_cc_proto",
        ":solution_manager",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_glog_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "sp2_solver",
    srcs = ["sp2_solver.cc"],
    hdrs = ["sp2_solver.h"],
    linkstatic = True,
    deps = [
        ":problem_cc_proto",
        ":problem_solver",
        ":solution_cc_proto",
        "@com_github_glog_glog//:glog",
    ],
)

cc_binary(
    name = "sp1_main",
    srcs = ["sp1_main.cc"],
    linkstatic = True,
    deps = [
        ":problem_cc_proto",
        ":problem_manager",
        ":problem_solver_factory",
        ":solution_cc_proto",
        ":solution_manager",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_glog_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "sp1_solver",
    srcs = ["sp1_solver.cc"],
    hdrs = ["sp1_solver.h"],
    linkstatic = True,
    deps = [
        ":problem_cc_proto",
        ":problem_solver",
        ":solution_cc_proto",
        "@com_github_glog_glog//:glog",
    ],
)

cc_library(
    name = "random_solver",
    srcs = ["random_solver.cc"],
    hdrs = ["random_solver.h"],
    linkstatic = True,
    deps = [
        ":problem_cc_proto",
        ":problem_solver",
        ":solution_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_glog_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "lp_solver",
    srcs = ["lp_solver.cc"],
    hdrs = ["lp_solver.h"],
    linkstatic = True,
    deps = [
        ":lp_solver_cc_proto",
        ":problem_cc_proto",
        ":problem_solver",
        ":solution_cc_proto",
        "@com_github_glog_glog//:glog",
        "@com_google_absl//absl/strings",
        "@third_party_ortools//ortools/base",
        "@third_party_ortools//ortools/linear_solver",
        "@third_party_ortools//ortools/linear_solver:linear_solver_cc_proto",
    ],
)

cc_proto_library(
    name = "lp_solver_cc_proto",
    deps = [":lp_solver_proto"],
)

proto_library(
    name = "lp_solver_proto",
    srcs = ["lp_solver.proto"],
)

cc_library(
    name = "problem_solver_factory",
    srcs = ["problem_solver_factory.cc"],
    hdrs = ["problem_solver_factory.h"],
    linkstatic = True,
    deps = [
        ":lp_solver",
        ":problem_solver",
        ":random_solver",
        ":sp1_solver",
        ":sp2_solver",
        "@com_github_glog_glog//:glog",
    ],
)

cc_library(
    name = "problem_solver",
    hdrs = ["problem_solver.h"],
    linkstatic = True,
    deps = [
        ":problem_cc_proto",
        ":solution_cc_proto",
    ],
)

cc_test(
    name = "solution_manager_test",
    srcs = ["solution_manager_test.cc"],
    deps = [
        ":solution_manager",
        "@gtest",
    ],
)

proto_library(
    name = "problem_proto",
    srcs = ["problem.proto"],
)

cc_proto_library(
    name = "problem_cc_proto",
    deps = [":problem_proto"],
)

cc_test(
    name = "problem_manager_test",
    srcs = ["problem_manager_test.cc"],
    deps = [
        ":problem_manager",
        "@gtest",
    ],
)

cc_library(
    name = "problem_manager",
    srcs = ["problem_manager.cc"],
    hdrs = ["problem_manager.h"],
    linkstatic = True,
    deps = [
        ":problem_cc_proto",
        "@com_github_glog_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

proto_library(
    name = "solution_proto",
    srcs = ["solution.proto"],
    deps = [":problem_proto"],
)

cc_proto_library(
    name = "solution_cc_proto",
    deps = [":solution_proto"],
)

cc_library(
    name = "solution_manager",
    srcs = ["solution_manager.cc"],
    hdrs = ["solution_manager.h"],
    linkstatic = True,
    deps = [
        ":solution_cc_proto",
        "@com_github_glog_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "main",
    srcs = ["main.cc"],
    linkstatic = True,
    deps = [
        ":problem_cc_proto",
        ":problem_manager",
        ":problem_solver_factory",
        ":solution_cc_proto",
        ":solution_manager",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_glog_glog//:glog",
    ],
)
