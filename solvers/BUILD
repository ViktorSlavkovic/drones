package(default_visibility = ["//visibility:public"])

cc_library(
    name = "problem_solver_factory",
    srcs = ["problem_solver_factory.cc"],
    hdrs = ["problem_solver_factory.h"],
    linkstatic = True,
    deps = [
        ":problem_solver",
        "//solvers/cwf_solver",
        "//solvers/ecf_solver",
        "//solvers/lp_solver",
        "//solvers/random_solver",
        "//solvers/sp1_solver",
        "//solvers/sp2_solver",
        "//solvers/sp4_solver",
        "@com_github_glog_glog//:glog",
    ],
)

cc_library(
    name = "problem_solver",
    hdrs = ["problem_solver.h"],
    linkstatic = True,
    deps = [
        "//:problem_cc_proto",
        "//:solution_cc_proto",
    ],
)
