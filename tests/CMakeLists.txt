add_executable(event_window_test event_window_test.cpp)

target_include_directories(event_window_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../app
    ${CMAKE_CURRENT_SOURCE_DIR}/../app/include
)

target_link_libraries(event_window_test PRIVATE GTest::GTest GTest::Main pthread)

add_test(NAME EventWindowTest COMMAND event_window_test)