cc_library(
    name = "glop_problem_solver",
    srcs = ["glop_problem_solver.cc"],
    hdrs = ["glop_problem_solver.h"],
    deps = [
        ":glop_problem_solver_cc_proto",
        ":problem_cc_proto",
        ":problem_solver",
        ":solution_cc_proto",
        "@com_github_glog_glog//:glog",
        "@third_party_ortools//ortools/base",
        "@third_party_ortools//ortools/linear_solver",
        "@third_party_ortools//ortools/linear_solver:linear_solver_cc_proto",
    ],
)

cc_proto_library(
    name = "glop_problem_solver_cc_proto",
    deps = [":glop_problem_solver_proto"],
)

proto_library(
    name = "glop_problem_solver_proto",
    srcs = ["glop_problem_solver.proto"],
)

cc_library(
    name = "problem_solver_factory",
    srcs = ["problem_solver_factory.cc"],
    hdrs = ["problem_solver_factory.h"],
    deps = [
        ":glop_problem_solver",
        ":problem_solver",
        "@com_github_glog_glog//:glog",
    ],
)

cc_library(
    name = "problem_solver",
    hdrs = ["problem_solver.h"],
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

cc_library(
    name = "problem_manager",
    srcs = ["problem_manager.cc"],
    hdrs = ["problem_manager.h"],
    deps = [
        ":problem_cc_proto",
        "@com_github_glog_glog//:glog",
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
    deps = [
        ":solution_cc_proto",
        "@com_github_glog_glog//:glog",
    ],
)

cc_binary(
    name = "main",
    srcs = ["main.cc"],
    linkstatic = True,
    deps = [
        ":problem_cc_proto",
        ":problem_manager",
        ":solution_cc_proto",
        ":solution_manager",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_glog_glog//:glog",
    ],
)

