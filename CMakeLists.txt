cmake_minimum_required(VERSION 3.2)

project(Cartesian_Impedance_Gen3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Needed for VS Code IntelliSense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ------------------------------------------------------------
# Compiler warnings
# ------------------------------------------------------------

if (MSVC)

    add_compile_options(
        /W4
        /WX
    )

else()

    add_compile_options(
        -Wall
        -Wextra
        -pedantic
    )

endif()

# ------------------------------------------------------------
# External dependencies
# ------------------------------------------------------------

find_package(PkgConfig REQUIRED)

pkg_check_modules(
    GTK3
    REQUIRED
    IMPORTED_TARGET
    gtk+-3.0
)

find_package(Eigen3 REQUIRED)

# ------------------------------------------------------------
# Custom CMake modules
# ------------------------------------------------------------

list(
    APPEND
    CMAKE_MODULE_PATH
    "${CMAKE_SOURCE_DIR}/cmake"
)

# ------------------------------------------------------------
# Existing laboratory dependencies
# ------------------------------------------------------------

add_subdirectory(
    Dependencies/Boost
)

add_subdirectory(
    Dependencies/Kinova
)

add_subdirectory(
    Matrix
)

add_subdirectory(
    KinovaRobot
)

add_subdirectory(
    DataLogging
)

# ============================================================
# CARTESIAN IMPEDANCE CONTROLLER LIBRARY
# ============================================================

add_library(
    cartesian_impedance

    src/cartesian_impedance_controller.cpp
)

target_include_directories(
    cartesian_impedance

    PUBLIC

    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    cartesian_impedance

    PUBLIC

    matrix
    Eigen3::Eigen
)

# ============================================================
# MAIN KINOVA IMPEDANCE EXECUTABLE
# ============================================================

add_executable(
    impedance_gen3

    src/main_impedance.cpp
)

target_include_directories(
    impedance_gen3

    PRIVATE

    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    impedance_gen3

    PRIVATE

    matrix
    kinova_robot
    kortex_api

    cartesian_impedance
    Eigen3::Eigen

    PkgConfig::GTK3
    data_logging
    Boost::numeric_odeint
)

# ============================================================
# ANALYTIC MODEL SMOKE TEST
# ============================================================

add_executable(
    model_smoke_test

    tests/model_smoke_test.cpp
)

target_include_directories(
    model_smoke_test

    PRIVATE

    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    model_smoke_test

    PRIVATE

    matrix
    Eigen3::Eigen
)

# ============================================================
# JACOBIAN VALIDATION TEST
# ============================================================

add_executable(
    jacobian_validation_test
    tests/jacobian_validation_test.cpp
)

target_include_directories(
    jacobian_validation_test
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    jacobian_validation_test
    PRIVATE
    matrix
)
