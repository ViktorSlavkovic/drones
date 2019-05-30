package(default_visibility = ["//visibility:public"])

load("@com_github_grpc_grpc//bazel:cc_grpc_library.bzl", "cc_grpc_library")

cc_library(
    name = "ga_solver",
    srcs = ["ga_solver.cc"],
    hdrs = ["ga_solver.h"],
    linkstatic = True,
    deps = [
        ":evaluator_service_cc_grpc",
        "//:problem_cc_proto",
        "//:solution_cc_proto",
        "//solvers:problem_solver",
        "//solvers/util:allocator",
        "//solvers/util:load_splitter",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_glog_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_github_grpc_grpc//:grpc++_unsecure",
    ],
)

cc_grpc_library(
  name = "evaluator_service_cc_grpc",
  srcs = [":evaluator_service_proto"],
  grpc_only = True,
  deps = [":evaluator_service_cc_proto"],
)

cc_proto_library(
  name = "evaluator_service_cc_proto",
  deps = [":evaluator_service_proto"],
)

proto_library(
  name = "evaluator_service_proto",
  srcs = ["evaluator_service.proto"],
)

