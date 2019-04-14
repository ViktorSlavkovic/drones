package(default_visibility = ["//visibility:public"])

load("@subpar//:subpar.bzl", "par_binary")

load(
  "@io_bazel_rules_python//python:python.bzl",
  "py_binary", "py_library", "py_test"
)

load("@external_python_deps//:requirements.bzl", "requirement")

par_binary(
  name = "visualize_problem",
  srcs = ["visualize_problem.py"],
  deps = [
    requirement("absl-py"),
    requirement("matplotlib"),
  ], 
)

par_binary(
  name = "visualize_solution",
  srcs = ["visualize_solution.py"],
  deps = [
    requirement("absl-py"),
    requirement("matplotlib"),
  ], 
)

