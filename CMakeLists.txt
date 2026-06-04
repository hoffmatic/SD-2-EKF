cmake_minimum_required(VERSION 3.16)

# Desktop build target for the AMBAR estimator/controller scaffold. The same
# source files are intended to stay portable when the STM32 project is added.
project(rocket_airbrake_ekf
    VERSION 0.1.0
    LANGUAGES CXX
)

# Keep the reusable estimator/controller code separate from executable entry
# points so demos and sandboxes all exercise the same core implementation.
add_library(ambar_core
    src/ambar_airbrake.cpp
)

target_compile_features(ambar_core PUBLIC cxx_std_17)
target_include_directories(ambar_core PUBLIC include)

if(MSVC)
    target_compile_options(ambar_core PRIVATE /W4)
else()
    target_compile_options(ambar_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Keep each executable small and explicit so new sandboxes must be intentionally
# added to the build.
add_executable(rocket_airbrake_ekf
    src/main.cpp
)

target_link_libraries(rocket_airbrake_ekf PRIVATE ambar_core)

add_executable(sim_flight_sandbox
    sim/flight_sandbox.cpp
)

target_link_libraries(sim_flight_sandbox PRIVATE ambar_core)

add_executable(sim_electronics_sandbox
    sim/electronics_sandbox.cpp
)

target_link_libraries(sim_electronics_sandbox PRIVATE ambar_core)

add_executable(sim_actuator_sandbox
    sim/actuator_sandbox.cpp
)

target_link_libraries(sim_actuator_sandbox PRIVATE ambar_core)

# Warnings are enabled on both MSVC and GCC/Clang-style compilers because this
# code will eventually control hardware and should be kept boringly clean.
if(MSVC)
    target_compile_options(rocket_airbrake_ekf PRIVATE /W4)
    target_compile_options(sim_flight_sandbox PRIVATE /W4)
    target_compile_options(sim_electronics_sandbox PRIVATE /W4)
    target_compile_options(sim_actuator_sandbox PRIVATE /W4)
else()
    target_compile_options(rocket_airbrake_ekf PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(sim_flight_sandbox PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(sim_electronics_sandbox PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(sim_actuator_sandbox PRIVATE -Wall -Wextra -Wpedantic)
endif()
