add_executable(world_test world_test.cpp)
target_link_libraries(world_test se3ez_core)

add_executable(fetch_plan fetch_plan.cpp)
target_link_libraries(fetch_plan se3ez_core)

add_executable(r2_plan r2_plan.cpp)
target_link_libraries(r2_plan se3ez_core)

add_executable(r2free_plan r2free_plan.cpp)
target_link_libraries(r2free_plan se3ez_core)

add_executable(fetch_robowflex_plan fetch_robowflex_plan.cpp)
target_link_libraries(fetch_robowflex_plan se3ez_core)

add_executable(r2con_plan r2con_plan.cpp)
target_link_libraries(r2con_plan se3ez_core)
