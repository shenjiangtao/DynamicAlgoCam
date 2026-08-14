add_executable(event_window_test event_window_test.cpp)

target_include_directories(event_window_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app
    ${CMAKE_CURRENT_SOURCE_DIR}/../app/include
)

target_link_libraries(event_window_test PRIVATE GTest::GTest GTest::Main pthread)

add_test(NAME EventWindowTest COMMAND event_window_test)

# --- detection_to_3d_test: exercises app/core/dynalgo_detection_to_3d.hpp.
# Header-only; test feeds hand-constructed DynalgoFrame/Intrinsic/DetectionResult
# (no SDK, no device) so the executable only links GTest + needs just the
# app/core include path.
add_executable(detection_to_3d_test detection_to_3d_test.cpp)

target_include_directories(detection_to_3d_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app/core
)

target_link_libraries(detection_to_3d_test PRIVATE GTest::GTest GTest::Main pthread)

add_test(NAME DetectionTo3DTest COMMAND detection_to_3d_test)
