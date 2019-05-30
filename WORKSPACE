load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

git_repository(
    name = "bazel_skylib",
    remote = "https://github.com/bazelbuild/bazel-skylib.git",
    commit = "b2b4471332abdbf8979c89ca5e2cbb4f3a2bdc0f",
)

git_repository(
    name = "third_party_ortools",
    remote = "https://github.com/google/or-tools.git",
    # branch="master",
    #commit="c4aaa4321d270d5d1a32ed8948ef565dca2ee3ff",
    commit = "39f44709bba203f5ff3bc18fab8098739f189a6d",
)

git_repository(
    name = "com_github_gflags_gflags",
    remote = "https://github.com/gflags/gflags.git",
    commit = "28f50e0fed19872e0fd50dd23ce2ee8cd759338e",
)

git_repository(
    name = "com_google_absl",
    remote = "https://github.com/abseil/abseil-cpp.git",
    #    commit = "7c7754fb3ed9ffb57d35fe8658f3ba4d73a31e72",
    commit = "a02f62f456f2c4a7ecf2be3104fe0c6e16fbad9a",
)

git_repository(
    name = "com_github_gflags_gflags",
    commit = "28f50e0fed19872e0fd50dd23ce2ee8cd759338e",
    remote = "https://github.com/gflags/gflags.git",
)

git_repository(
    name = "com_github_glog_glog",
    remote = "https://github.com/google/glog.git",
    commit = "5c576f78c49b28d89b23fbb1fc80f54c879ec02e",
)

http_archive(
    name = "com_google_protobuf",
    urls = ["https://github.com/google/protobuf/archive/8e5ea65953f3c47e01bca360ecf3abdf2c8b1c33.zip"],
    strip_prefix = "protobuf-8e5ea65953f3c47e01bca360ecf3abdf2c8b1c33",
    #    urls = ["https://github.com/google/protobuf/archive/b68a347f56137b4b1a746e8c7438495a6ac1bd91.zip"],
    #    strip_prefix = "protobuf-b68a347f56137b4b1a746e8c7438495a6ac1bd91",
)

http_archive(
    name = "com_google_protobuf_cc",
    urls = ["https://github.com/google/protobuf/archive/8e5ea65953f3c47e01bca360ecf3abdf2c8b1c33.zip"],
    strip_prefix = "protobuf-8e5ea65953f3c47e01bca360ecf3abdf2c8b1c33",
    #    urls = ["https://github.com/google/protobuf/archive/b68a347f56137b4b1a746e8c7438495a6ac1bd91.zip"],
    #    strip_prefix = "protobuf-b68a347f56137b4b1a746e8c7438495a6ac1bd91",
)

http_archive(
    name = "com_github_grpc_grpc",
    urls = [
        "https://github.com/grpc/grpc/archive/6354b810278409d57e2900dfb3532e656e96851d.tar.gz",
    ],
    strip_prefix = "grpc-6354b810278409d57e2900dfb3532e656e96851d",
)

load("@com_github_grpc_grpc//bazel:grpc_deps.bzl", "grpc_deps")

grpc_deps()

git_repository(
    name = "gtest",
    commit = "8b6d3f9c4a774bef3081195d422993323b6bb2e0",
    remote = "https://github.com/google/googletest.git",
)

http_archive(
    name = "glpk",
    url = "http://ftp.gnu.org/gnu/glpk/glpk-4.52.tar.gz",
    sha256 = "9a5dab356268b4f177c33e00ddf8164496dc2434e83bd1114147024df983a3bb",
    build_file = "//bazel:glpk.BUILD",
)

http_archive(
    name = "net_zlib",
    build_file_content = """
licenses(["notice"])  #  BSD/MIT-like license

filegroup(
    name = "srcs",
    srcs = glob(["**"]),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "embedded_tools",
    srcs = glob(["*.c"]) + glob(["*.h"]) + ["BUILD"] + ["LICENSE.txt"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "zlib",
    srcs = glob(["*.c"]),
    hdrs = glob(["*.h"]),
    # Use -Dverbose=-1 to turn off zlib's trace logging. (#3280)
    copts = ["-w", "-Dverbose=-1"],
    includes = ["."],
    visibility = ["//visibility:public"],
)
    """,
    sha256 = "c3e5e9fdd5004dcb542feda5ee4f0ff0744628baf8ed2dd5d66f8ca1197cb1a1",
    strip_prefix = "zlib-1.2.11",
    urls = ["https://zlib.net/zlib-1.2.11.tar.gz"],
)

bind(
    name = "zlib",
    actual = "@net_zlib//:zlib",
)

###############################################################################
# Python stuff
###############################################################################

git_repository(
    name = "subpar",
    remote = "https://github.com/google/subpar",
    commit = "0356bef3fbbabec5f0e196ecfacdeb6db62d48c0",
)

git_repository(
    name = "io_bazel_rules_python",
    remote = "https://github.com/bazelbuild/rules_python.git",
    commit = "fa6ab781188972aa2710310b29a9bccaae7fd7fe",
)

load("@io_bazel_rules_python//python:pip.bzl", "pip_repositories")

pip_repositories()

load("@io_bazel_rules_python//python:pip.bzl", "pip_import")

pip_import(
    name = "external_python_deps",
    requirements = "//:.python_external_deps",
)

load("@external_python_deps//:requirements.bzl", "pip_install")

pip_install()
