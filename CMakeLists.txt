cmake_minimum_required(VERSION 3.16)

project(rocket_airbrake_ekf
    VERSION 0.1.0
    LANGUAGES CXX
)

add_executable(rocket_airbrake_ekf
    src/main.cpp
)

target_compile_features(rocket_airbrake_ekf PRIVATE cxx_std_17)

if(MSVC)
    target_compile_options(rocket_airbrake_ekf PRIVATE /W4)
else()
    target_compile_options(rocket_airbrake_ekf PRIVATE -Wall -Wextra -Wpedantic)
endif()
