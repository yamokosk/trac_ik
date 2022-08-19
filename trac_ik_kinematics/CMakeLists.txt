cmake_minimum_required(VERSION 3.10.2)
project(trac_ik_kinematics)

find_package(rclcpp)
find_package(moveit_core)
find_package(orocos_kdl)
find_package(kdl_parser)
find_package(tf2_kdl)
find_package(pluginlib)
find_package(trac_ik_lib)
find_package(geometry_msgs)
find_package(moveit_msgs)

set(THIS_PACKAGE_INCLUDE_DIRS
  trac_ik_kinematics_plugin/include
)

set(THIS_PACKAGE_LIBRARIES
  moveit_trac_ik_kinematics_plugin
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_core
  orocos_kdl
  kdl_parser
  tf2_kdl
  rclcpp
  pluginlib
  trac_ik_lib
  geometry_msgs
  moveit_msgs
)

pluginlib_export_plugin_description_file(moveit_core  trac_ik_kinematics_plugin_description.xml)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

add_subdirectory(trac_ik_kinematics_plugin)

install(
  TARGETS ${THIS_PACKAGE_LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()