add_executable(shell-app main.cpp sensor.cpp robot.cpp controller.cpp forwardkinematics.cpp inversekinematics.cpp pbPlots.cpp supportLib.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
