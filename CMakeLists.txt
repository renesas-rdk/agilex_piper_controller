cmake_minimum_required(VERSION 3.8)
project(agilex_piper_controller)

# Build settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-psabi)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# Library
add_library(${PROJECT_NAME} SHARED
  src/can_interface.cpp
  src/piper_controller.cpp
  src/piper_messages.cpp
  src/piper_params.cpp
  src/piper_protocol.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    src
)

# Link dependencies
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${EIGEN3_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} SYSTEM
  PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
)

# Installation
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()