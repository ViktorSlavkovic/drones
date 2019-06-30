package(default_visibility = ["//visibility:public"])

load("@subpar//:subpar.bzl", "par_binary")
load(
    "@io_bazel_rules_python//python:python.bzl",
    "py_binary",
    "py_library",
    "py_test",
)
load("@external_python_deps//:requirements.bzl", "requirement")

par_binary(
  name = "batch_runner",
  srcs = ["batch_runner.py"],
  data = [":main"],
  deps = [
    requirement("absl-py"),
    requirement("numpy"),
  ],
)

par_binary(
  name = "batch_stats",
  srcs = ["batch_stats.py"],
  deps = [
    requirement("absl-py"),
    requirement("numpy"),
  ],
)

sh_binary(
    name = "sp5_main",
    srcs = ["sp5_main.sh"],
    deps = [
        ":run_util",
    ],
)

sh_binary(
    name = "sp4_main",
    srcs = ["sp4_main.sh"],
    deps = [
        ":run_util",
    ],
)

sh_binary(
    name = "sp3_main",
    srcs = ["sp3_main.sh"],
    deps = [
        ":run_util",
    ],
)

sh_binary(
    name = "sp2_main",
    srcs = ["sp2_main.sh"],
    deps = [
        ":run_util",
    ],
)

sh_binary(
    name = "sp1_main",
    srcs = ["sp1_main.sh"],
    deps = [
        ":run_util",
    ],
)

sh_library(
    name = "run_util",
    srcs = ["run_util.sh"],
    data = [
        ":main",
        "//checker:main",
        "//visual:visualize_solution",
    ],
)

cc_binary(
    name = "generator",
    srcs = ["generator.cc"],
    linkstatic = True,
    deps = [
        ":problem_cc_proto",
        ":problem_manager",
        "//solvers/upper_bound_higher",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_glog_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_protobuf_cc//:protobuf",
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
        "//solvers:problem_solver_factory",
        "//solvers/upper_bound",
        "//solvers/upper_bound_higher",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_glog_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@com_google_protobuf_cc//:protobuf",
    ],
)

cc_test(
    name = "solution_manager_test",
    srcs = ["solution_manager_test.cc"],
    deps = [
        ":problem_manager",
        ":solution_manager",
        "//solvers:problem_solver_factory",
        "@gtest",
    ],
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

cc_proto_library(
    name = "solution_cc_proto",
    deps = [":solution_proto"],
)

proto_library(
    name = "solution_proto",
    srcs = ["solution.proto"],
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

cc_proto_library(
    name = "problem_cc_proto",
    deps = [":problem_proto"],
)

proto_library(
    name = "problem_proto",
    srcs = ["problem.proto"],
)
