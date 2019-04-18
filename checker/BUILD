package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "main",
    srcs = ["main.cc"],
    # These options are overriding the ones in .bazelrc in order to prevent the
    # compiler from optimizing out the assertions. Sure glog's CHECKs would be
    # better, but this binary can't have dependencies in case it's to be used
    # as a checker on some online judge (though, in that case, the assertions
    # might have to go away).
    copts = [
        "-g",
        "-Og",
        "-UNDEBUG",
    ],
    linkstatic = True,
)
