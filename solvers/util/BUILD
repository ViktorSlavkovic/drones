package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lp_util",
    srcs = ["lp_util.cc"],
    hdrs = ["lp_util.h"],
    linkstatic = True,
    deps = [
        "@com_github_glog_glog//:glog",
        "@com_google_protobuf_cc//:protobuf",
    ],
)

cc_library(
    name = "allocator",
    srcs = ["allocator.cc"],
    hdrs = ["allocator.h"],
    linkstatic = True,
    deps = [
        ":alloc_cc_proto",
        ":lp_util",
        "//:problem_cc_proto",
        "@com_github_glog_glog//:glog",
        "@third_party_ortools//ortools/base",
        "@third_party_ortools//ortools/linear_solver",
        "@third_party_ortools//ortools/linear_solver:linear_solver_cc_proto",
    ],
)

cc_proto_library(
    name = "alloc_cc_proto",
    deps = [":alloc_proto"],
)

proto_library(
    name = "alloc_proto",
    srcs = ["alloc.proto"],
)
