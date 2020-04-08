
set(prjname phd_member)
set(version 1)

project(${prjname} CXX)
add_subdirectory(jail)
add_subdirectory(math)
add_subdirectory(ang)
add_subdirectory(ang_test)
add_subdirectory(env)
add_subdirectory(env_test)
add_subdirectory(acft)
add_subdirectory(acft_test)
add_subdirectory(nav)
add_subdirectory(nav_test)
add_subdirectory(results_sim_test)
add_subdirectory(results_ana_test)
add_subdirectory(exec_nav)
add_subdirectory(exec_gps_bench)
add_subdirectory(exec_gps_bench_mult)
add_subdirectory(exec_gps_alter)
add_subdirectory(exec_gps_alter_mult)
add_subdirectory(exec_cntr)




