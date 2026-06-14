cmake_minimum_required(VERSION 3.16)

# Architecture role: describes the native C++ dependency graph. ambar_core is
# the shared flight-logic library; every executable below is a desktop adapter,
# demo, simulation suite, or process bridge around that library.

# Desktop build target for the AMBAR estimator/controller scaffold. The same
# source files are intended to stay portable when the STM32 project is added.
project(rocket_airbrake_ekf
    VERSION 0.1.0
    LANGUAGES CXX
)

include(CTest)

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

add_executable(ambar_controller_bridge
    sim/controller_bridge.cpp
)

target_link_libraries(ambar_controller_bridge PRIVATE ambar_core)

add_executable(sim_fault_replay_sandbox
    sim/fault_replay_sandbox.cpp
)

target_link_libraries(sim_fault_replay_sandbox PRIVATE ambar_core)

add_executable(sim_monte_carlo_sandbox
    sim/monte_carlo_sandbox.cpp
)

target_link_libraries(sim_monte_carlo_sandbox PRIVATE ambar_core)

add_executable(ambar_core_tests
    tests/ambar_core_tests.cpp
)

target_link_libraries(ambar_core_tests PRIVATE ambar_core)

# Warnings are enabled on both MSVC and GCC/Clang-style compilers because this
# code will eventually control hardware and should be kept boringly clean.
if(MSVC)
    target_compile_options(rocket_airbrake_ekf PRIVATE /W4)
    target_compile_options(sim_flight_sandbox PRIVATE /W4)
    target_compile_options(sim_electronics_sandbox PRIVATE /W4)
    target_compile_options(sim_actuator_sandbox PRIVATE /W4)
    target_compile_options(ambar_controller_bridge PRIVATE /W4)
    target_compile_options(sim_fault_replay_sandbox PRIVATE /W4)
    target_compile_options(sim_monte_carlo_sandbox PRIVATE /W4)
    target_compile_options(ambar_core_tests PRIVATE /W4)
else()
    target_compile_options(rocket_airbrake_ekf PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(sim_flight_sandbox PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(sim_electronics_sandbox PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(sim_actuator_sandbox PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(ambar_controller_bridge PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(sim_fault_replay_sandbox PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(sim_monte_carlo_sandbox PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(ambar_core_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING)
    add_test(NAME ambar_core_tests COMMAND ambar_core_tests)
    add_test(NAME flight_sandbox COMMAND sim_flight_sandbox)
    add_test(NAME electronics_sandbox COMMAND sim_electronics_sandbox)
    add_test(NAME actuator_sandbox COMMAND sim_actuator_sandbox)
    add_test(NAME fault_replay_sandbox COMMAND sim_fault_replay_sandbox)
    add_test(NAME monte_carlo_sandbox COMMAND sim_monte_carlo_sandbox)
    set_tests_properties(fault_replay_sandbox PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
endif()
