# tests for regulated PP
ament_add_gtest(test_scan_rpp test_scan_rpp.cpp)
ament_target_dependencies(test_scan_rpp ${dependencies})
target_link_libraries(test_scan_rpp ${library_name})

ament_add_gtest(test_costmap_path_smoother test_costmap_path_smoother.cpp)
ament_target_dependencies(test_costmap_path_smoother ${dependencies})
target_link_libraries(test_costmap_path_smoother ${library_name})

ament_add_gtest(test_regulated_pp
  test_regulated_pp.cpp
  path_utils/path_utils.cpp
)
ament_target_dependencies(test_regulated_pp
  ${dependencies}
)
target_link_libraries(test_regulated_pp
  ${library_name}
)

# Path utils test
ament_add_gtest(test_path_utils path_utils/test_path_utils.cpp path_utils/path_utils.cpp)
ament_target_dependencies(test_path_utils nav_msgs geometry_msgs tf2_geometry_msgs)
