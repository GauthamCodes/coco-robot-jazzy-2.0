cmake_minimum_required(VERSION 3.5)
project(coco_nav_diag)

# C2-NAV.36 diagnostic package. Builds a separately-named fork of
# nav2_amcl's AmclNode (coco_nav_diag::AmclNode, executable `amcl_diag`)
# with default-disabled instrumentation of the scan-stamped odom TF lookup
# and motion-model delta computation. Deliberately does NOT rebuild
# nav2_amcl's pf/map/motion_model/sensors libraries -- only amcl_node.cpp
# is recompiled; the rest links against the already-installed nav2_amcl
# package (headers + libmap_lib.so/libpf_lib.so/libsensors_lib.so/
# libmotions_lib.so), exactly as the stock nav2_amcl amcl_core library
# does. See docs/agents/HANDOFF.md for the full rationale.

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(Threads REQUIRED)

nav2_package()

include_directories(
  include
)

# Mirrors upstream nav2_amcl/CMakeLists.txt: portable_utils.hpp (pulled in
# transitively via amcl_node.cpp -> nav2_util headers on this glibc target)
# only defines its drand48()/srand48() polyfill when HAVE_DRAND48 is unset.
# glibc already declares both extern in stdlib.h, so without this check the
# polyfill's static redeclaration conflicts with the libc one.
include(CheckSymbolExists)
check_symbol_exists(drand48 stdlib.h HAVE_DRAND48)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  message_filters
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  tf2_ros
  tf2
  nav2_util
  nav2_msgs
  pluginlib
  nav2_amcl
)

set(executable_name amcl_diag)
set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/amcl_node.cpp
  src/diag_recorder.cpp
)

if(HAVE_DRAND48)
  target_compile_definitions(${library_name} PRIVATE "HAVE_DRAND48")
endif()

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${library_name}
  Threads::Threads
)

add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name}
  ${library_name}
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # C2-NAV.36: DiagRecorder is deliberately ROS-free (see
  # diag_recorder.hpp), so its test binary needs no ament_target_dependencies
  # at all -- only the two source files and pthread.
  ament_add_gtest(test_diag_recorder
    test/test_diag_recorder.cpp
    src/diag_recorder.cpp
  )
  target_include_directories(test_diag_recorder PRIVATE include)
  # ament_add_gtest already links GTEST_MAIN_LIBRARIES and (non-Windows)
  # Threads::Threads for us.

  # C2-NAV.37 pre-flight: unlike test_diag_recorder above, this test
  # constructs the real coco_nav_diag::AmclNode (the class amcl_diag's
  # main() must instantiate) to catch a regression of the kind that made
  # amcl_diag silently run plain, uninstrumented AMCL. Needs the full
  # nav2_amcl dependency set, same as amcl_diag_core/amcl_diag above.
  ament_add_gtest(test_amcl_diag_node
    test/test_amcl_diag_node.cpp
  )
  target_include_directories(test_amcl_diag_node PRIVATE include)
  target_link_libraries(test_amcl_diag_node ${library_name})
  ament_target_dependencies(test_amcl_diag_node ${dependencies})
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
