set(GTEST_SHUFFLE 1)

add_executable(
    cpp-test
    main.cpp
    robot_test.cpp
    sensor_test.cpp
    controller_test.cpp
    forwardkinematics_test.cpp
    ../app/sensor.cpp
    ../app/robot.cpp
    ../app/controller.cpp
    ../app/forwardkinematics.cpp

)

target_include_directories(cpp-test PUBLIC ../vendor/googletest/googletest/include 
                                           ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(cpp-test PUBLIC gtest)
