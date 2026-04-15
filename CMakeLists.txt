cmake_minimum_required(VERSION 3.10)
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
    Eigen3::Eigen
)

enable_testing()
add_subdirectory(tests)

# Installation
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  EXPORT ${PROJECT_NAME}Targets
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
)