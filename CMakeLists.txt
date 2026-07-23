cmake_minimum_required(VERSION 3.16)

# Architecture role: builds two explicitly different desktop paths. ambar_core
# supports the legacy C++ regression sandboxes; ambar_stm32_controller_bridge
# compiles the production STM32-C EKF/flight modules used by RocketPy.

# Desktop targets for legacy regression fixtures and the production-C bridge.
project(rocket_airbrake_ekf
    VERSION 0.1.0
    LANGUAGES C CXX
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
set_target_properties(ambar_controller_bridge PROPERTIES
    FOLDER "legacy-cpp-sandboxes"
)

# Production-C bridge used by RocketPy and the Monte Carlo study.  Keeping this
# target separate from the older C++ scaffold makes it explicit which runs use
# the same estimator/flight sources that are compiled into the STM32 firmware.
add_executable(ambar_stm32_controller_bridge
    firmware/stm32_airbrake_pcb/Core/Src/ambar_ekf.c
    firmware/stm32_airbrake_pcb/Core/Src/ambar_flight.c
    sim/stm32_controller_bridge.c
)

target_compile_features(ambar_stm32_controller_bridge PRIVATE c_std_11)
target_include_directories(ambar_stm32_controller_bridge PRIVATE
    firmware/stm32_airbrake_pcb/Core/Inc
)
if(NOT MSVC)
    target_link_libraries(ambar_stm32_controller_bridge PRIVATE m)
endif()

# Host tests for production STM32 modules. These targets intentionally compile
# the same C sources as CubeIDE while replacing only board-specific hardware at
# the test boundary.
add_executable(ambar_rocket_protocol_tests
    firmware/stm32_airbrake_pcb/Core/Src/rocket_protocol.c
    firmware/stm32_airbrake_pcb/HostTests/rocket_protocol_tests.c
)
target_compile_features(ambar_rocket_protocol_tests PRIVATE c_std_11)
target_include_directories(ambar_rocket_protocol_tests PRIVATE
    firmware/stm32_airbrake_pcb/Core/Inc
)

# The VARIABLE_HIL application consumes these pure decisions for arming and
# session teardown.  Keeping the policy HAL-free lets the host suite exercise
# stale-input and USB-loss behavior directly rather than relying on source-text
# checks around the embedded application loop.
add_executable(ambar_variable_hil_policy_tests
    firmware/stm32_airbrake_pcb/Core/Src/ambar_variable_hil_policy.c
    firmware/stm32_airbrake_pcb/HostTests/variable_hil_policy_tests.c
)
target_compile_features(ambar_variable_hil_policy_tests PRIVATE c_std_11)
target_include_directories(ambar_variable_hil_policy_tests PRIVATE
    firmware/stm32_airbrake_pcb/Core/Inc
)

# A V2 flash record must migrate into the expanded V3 control structure in RAM
# without an automatic write.  This target compiles the production config
# implementation and supplies only flash/flight/actuator fakes at its boundary.
# ST's CMSIS device header explicitly rejects MSVC's preprocessor, so this one
# target is built by GCC/Clang hosts (including CI and the local MinGW run).
if(NOT MSVC)
    add_executable(ambar_config_migration_tests
        firmware/stm32_airbrake_pcb/Core/Src/ambar_config.c
        firmware/stm32_airbrake_pcb/HostTests/ambar_config_migration_tests.c
    )
    target_compile_features(ambar_config_migration_tests PRIVATE c_std_11)
    target_compile_definitions(ambar_config_migration_tests PRIVATE
        STM32H562xx
        USE_HAL_DRIVER
    )
    target_include_directories(ambar_config_migration_tests PRIVATE
        firmware/stm32_airbrake_pcb/Core/Inc
        firmware/stm32_airbrake_pcb/Drivers/STM32H5xx_HAL_Driver/Inc
        firmware/stm32_airbrake_pcb/Drivers/STM32H5xx_HAL_Driver/Inc/Legacy
        firmware/stm32_airbrake_pcb/Drivers/CMSIS/Device/ST/STM32H5xx/Include
        firmware/stm32_airbrake_pcb/Drivers/CMSIS/Include
    )
    target_link_libraries(ambar_config_migration_tests PRIVATE m)
endif()

add_executable(ambar_actuator_autoretract_tests
    firmware/stm32_airbrake_pcb/Core/Src/ambar_actuator.c
    firmware/stm32_airbrake_pcb/HostTests/actuator_autoretract_tests.c
)
target_compile_features(ambar_actuator_autoretract_tests PRIVATE c_std_11)
target_compile_definitions(ambar_actuator_autoretract_tests PRIVATE
    AMBAR_BUILD_PROFILE=AMBAR_BUILD_PROFILE_NORMAL
)
target_include_directories(ambar_actuator_autoretract_tests PRIVATE
    firmware/stm32_airbrake_pcb/HostTests/actuator_fakes
    firmware/stm32_airbrake_pcb/Core/Inc
)
if(NOT MSVC)
    target_link_libraries(ambar_actuator_autoretract_tests PRIVATE m)
endif()

add_executable(ambar_actuator_hil_limit_tests
    firmware/stm32_airbrake_pcb/Core/Src/ambar_actuator.c
    firmware/stm32_airbrake_pcb/HostTests/actuator_hil_limit_tests.c
)
target_compile_features(ambar_actuator_hil_limit_tests PRIVATE c_std_11)
target_compile_definitions(ambar_actuator_hil_limit_tests PRIVATE
    AMBAR_BUILD_PROFILE=AMBAR_BUILD_PROFILE_CONTINUOUS_HIL
)
target_include_directories(ambar_actuator_hil_limit_tests PRIVATE
    firmware/stm32_airbrake_pcb/HostTests/actuator_fakes
    firmware/stm32_airbrake_pcb/Core/Inc
)
if(NOT MSVC)
    target_link_libraries(ambar_actuator_hil_limit_tests PRIVATE m)
endif()

# VARIABLE_HIL uses the production fractional controller-to-actuator path while
# retaining HIL-only simulation/freshness gates.  Compiling the same actuator
# source under the third profile is an explicit isolation regression check.
add_executable(ambar_actuator_variable_hil_tests
    firmware/stm32_airbrake_pcb/Core/Src/ambar_actuator.c
    firmware/stm32_airbrake_pcb/HostTests/actuator_autoretract_tests.c
)
target_compile_features(ambar_actuator_variable_hil_tests PRIVATE c_std_11)
target_compile_definitions(ambar_actuator_variable_hil_tests PRIVATE
    AMBAR_BUILD_PROFILE=AMBAR_BUILD_PROFILE_VARIABLE_HIL
)
target_include_directories(ambar_actuator_variable_hil_tests PRIVATE
    firmware/stm32_airbrake_pcb/HostTests/actuator_fakes
    firmware/stm32_airbrake_pcb/Core/Inc
)
if(NOT MSVC)
    target_link_libraries(ambar_actuator_variable_hil_tests PRIVATE m)
endif()

add_executable(ambar_flight_profile_replay
    firmware/stm32_airbrake_pcb/Core/Src/ambar_ekf.c
    firmware/stm32_airbrake_pcb/Core/Src/ambar_flight.c
    firmware/stm32_airbrake_pcb/HostTests/flight_profile_replay.c
)
target_compile_features(ambar_flight_profile_replay PRIVATE c_std_11)
target_include_directories(ambar_flight_profile_replay PRIVATE
    firmware/stm32_airbrake_pcb/Core/Inc
)
if(NOT MSVC)
    target_link_libraries(ambar_flight_profile_replay PRIVATE m)
endif()

add_executable(ambar_flight_predictive_solver_tests
    firmware/stm32_airbrake_pcb/Core/Src/ambar_ekf.c
    firmware/stm32_airbrake_pcb/Core/Src/ambar_flight.c
    firmware/stm32_airbrake_pcb/HostTests/flight_predictive_solver_tests.c
)
target_compile_features(ambar_flight_predictive_solver_tests PRIVATE c_std_11)
target_include_directories(ambar_flight_predictive_solver_tests PRIVATE
    firmware/stm32_airbrake_pcb/Core/Inc
)
if(NOT MSVC)
    target_link_libraries(ambar_flight_predictive_solver_tests PRIVATE m)
endif()

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
    target_compile_options(ambar_stm32_controller_bridge PRIVATE /W4)
    # rocket_protocol_tests.c uses the standard assert() macro; keep those
    # checks active even when CI builds the Release configuration.
    target_compile_options(ambar_rocket_protocol_tests PRIVATE /W4 /UNDEBUG)
    target_compile_options(ambar_variable_hil_policy_tests PRIVATE /W4)
    target_compile_options(ambar_actuator_autoretract_tests PRIVATE /W4)
    target_compile_options(ambar_actuator_hil_limit_tests PRIVATE /W4)
    target_compile_options(ambar_actuator_variable_hil_tests PRIVATE /W4)
    target_compile_options(ambar_flight_profile_replay PRIVATE /W4)
    target_compile_options(ambar_flight_predictive_solver_tests PRIVATE /W4)
    target_compile_options(sim_fault_replay_sandbox PRIVATE /W4)
    target_compile_options(sim_monte_carlo_sandbox PRIVATE /W4)
    target_compile_options(ambar_core_tests PRIVATE /W4)
else()
    target_compile_options(rocket_airbrake_ekf PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(sim_flight_sandbox PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(sim_electronics_sandbox PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(sim_actuator_sandbox PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(ambar_controller_bridge PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(ambar_stm32_controller_bridge PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(ambar_rocket_protocol_tests PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
    target_compile_options(ambar_variable_hil_policy_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(ambar_config_migration_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(ambar_actuator_autoretract_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(ambar_actuator_hil_limit_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(ambar_actuator_variable_hil_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(ambar_flight_profile_replay PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(ambar_flight_predictive_solver_tests PRIVATE -Wall -Wextra -Wpedantic)
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
    add_test(NAME stm32_rocket_protocol COMMAND ambar_rocket_protocol_tests)
    add_test(NAME stm32_variable_hil_policy COMMAND ambar_variable_hil_policy_tests)
    if(NOT MSVC)
        add_test(NAME stm32_config_migration COMMAND ambar_config_migration_tests)
    endif()
    add_test(NAME stm32_actuator_autoretract COMMAND ambar_actuator_autoretract_tests)
    add_test(NAME stm32_actuator_hil_limits COMMAND ambar_actuator_hil_limit_tests)
    add_test(NAME stm32_actuator_variable_hil COMMAND ambar_actuator_variable_hil_tests)
    add_test(NAME stm32_flight_predictive_solver COMMAND ambar_flight_predictive_solver_tests)
    add_test(
        NAME stm32_flight_profile_replay
        COMMAND ambar_flight_profile_replay
            "${CMAKE_SOURCE_DIR}/firmware/stm32_airbrake_pcb/SimulationData/26-07-10_Openrocket_Run_50Hz_DERIVED.csv"
    )
    set_tests_properties(fault_replay_sandbox PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
endif()
