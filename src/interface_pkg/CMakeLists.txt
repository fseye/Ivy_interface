cmake_minimum_required(VERSION 3.8)
project(interface_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Active automoc, autouic, autorcc pour Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets UiTools)

install(FILES resource/mainwindow.ui DESTINATION share/${PROJECT_NAME}/resource)

add_executable(ivy_interface
  src/interface_node.cpp
)

ament_target_dependencies(ivy_interface rclcpp std_msgs)

target_link_libraries(ivy_interface
  Qt5::Widgets
  Qt5::UiTools
)

include_directories(
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5UiTools_INCLUDE_DIRS}
)

install(TARGETS
  ivy_interface
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
