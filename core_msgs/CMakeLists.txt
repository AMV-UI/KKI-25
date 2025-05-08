cmake_minimum_required(VERSION 3.8)
project(core_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidL_generate_interfaces(${PROJECT_NAME} 
"msg/AutoControl.msg"
"msg/Camera.msg"
"msg/Config.msg"
"msg/Controller.msg"
"msg/Detection.msg"
"msg/KillSwitch.msg"
"msg/Mission.msg"
"msg/MuxState.msg"
"msg/ObjectCount.msg"
"msg/Option.msg"
"msg/Pwm.msg"
"msg/Pixhawk.msg"
"msg/StateObject.msg"

)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
