cmake_minimum_required(VERSION 3.5)
project(zivid_parameter_server)

if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
  add_compile_options(-std=c++17)
else()
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option(COMPILER_WARNINGS
       "Turn on warnings and warnings-as-errors during compilation" OFF)
if(COMPILER_WARNINGS)
  include(CompilerWarnings NO_POLICY_SCOPE)
endif()

function(turn_on_compiler_warnings_if_enabled TARGET)
  if(COMPILER_WARNINGS)
    set_target_warning_compile_options(${TARGET})
  endif()
endfunction()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(Zivid 1.8.0 COMPONENTS Core REQUIRED)
message(STATUS "Found Zivid version ${Zivid_VERSION}")

add_library(zivid_parameter_server_library SHARED
            src/zivid_parameter_server.cpp)
turn_on_compiler_warnings_if_enabled(zivid_parameter_server_library)
target_include_directories(zivid_parameter_server_library PRIVATE include)
target_compile_definitions(
  zivid_parameter_server_library PRIVATE
  "ZIVID_ROS_DRIVER_VERSION=\"${${PROJECT_NAME}_VERSION}\"")
target_compile_definitions(zivid_parameter_server_library PRIVATE
                           "ZIVID_PARAMETER_SERVER_BUILDING_DLL")
target_link_libraries(zivid_parameter_server_library Zivid::Core)
ament_target_dependencies(zivid_parameter_server_library rclcpp
                          rclcpp_components)
rclcpp_components_register_node(zivid_parameter_server_library
                                PLUGIN
                                "zivid_parameter_server::ZividParameterServer"
                                EXECUTABLE
                                "zivid_parameter_server")

# since the package installs libraries without exporting them it needs to make
# sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(TARGETS zivid_parameter_server_library
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
