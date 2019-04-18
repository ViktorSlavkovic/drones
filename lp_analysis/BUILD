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
    name = "var_completeness",
    srcs = ["var_completeness.py"],
    data = [":eval_main"],
    deps = [
        requirement("absl-py"),
        requirement("numpy"),
        requirement("matplotlib"),
        requirement("psopy"),
        requirement("scipy"),
    ],
)

cc_binary(
    name = "eval_main",
    srcs = ["eval_main.cc"],
    linkstatic = True,
    deps = [
        "@com_github_gflags_gflags//:gflags",
        "@com_github_glog_glog//:glog",
    ],
)
