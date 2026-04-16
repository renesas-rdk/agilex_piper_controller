add_executable(test1 test1.cpp)

target_link_libraries(test1
  PRIVATE
    agilex_piper_controller
)

add_executable(test_can_max_rate test_can_max_rate.cpp)

target_link_libraries(test_can_max_rate
  PRIVATE
    agilex_piper_controller
)

add_executable(test_mit_mode test_mit_mode.cpp)

target_link_libraries(test_mit_mode
  PRIVATE
    agilex_piper_controller
)
