cmake_minimum_required(VERSION 3.16)

# Desktop build target for the AMBAR estimator/controller scaffold. The same
# source files are intended to stay portable when the STM32 project is added.
project(rocket_airbrake_ekf
    VERSION 0.1.0
    LANGUAGES CXX
)

# Keep the executable small and explicit so new files must be intentionally
# added to the build.
add_executable(rocket_airbrake_ekf
    src/ambar_airbrake.cpp
    src/main.cpp
)

# C++17 is enough for fixed-size arrays and constexpr constants without pulling
# in heavier modern-library dependencies that may complicate embedded builds.
target_compile_features(rocket_airbrake_ekf PRIVATE cxx_std_17)
target_include_directories(rocket_airbrake_ekf PRIVATE include)

# Warnings are enabled on both MSVC and GCC/Clang-style compilers because this
# code will eventually control hardware and should be kept boringly clean.
if(MSVC)
    target_compile_options(rocket_airbrake_ekf PRIVATE /W4)
else()
    target_compile_options(rocket_airbrake_ekf PRIVATE -Wall -Wextra -Wpedantic)
endif()
