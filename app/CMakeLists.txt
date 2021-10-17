add_executable(shell-app main.cpp sensor.cpp robotkinematics.cpp controller.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
