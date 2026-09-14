cmake_minimum_required(VERSION 3.22)
project(KronMotion C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(KRONMOTION_BUILD_TESTS "Build the KronMotion self test"    ON)
option(KRONMOTION_BUILD_TOOL  "Build the ImGui/ImPlot GUI tool"   OFF)

# ── The trajectory core, fetched from its own repository ────────────────────
# SOURCE_SUBDIR points at a directory that holds no CMakeLists, so FetchContent
# downloads the repository without configuring it: its own CMakeLists requires
# Qt6 for the test rig, which has no business in a controller build. The two
# core files are compiled straight into the kronmotion library below.
include(FetchContent)

FetchContent_Declare(onlinetrajectorygenerator
    GIT_REPOSITORY https://github.com/fehimkus/OnlineTrajectoryGenerator.git
    GIT_TAG        main
    GIT_SHALLOW    TRUE
    SOURCE_SUBDIR  no-cmake-here
)
FetchContent_MakeAvailable(onlinetrajectorygenerator)

set(MOTION_CORE_DIR ${onlinetrajectorygenerator_SOURCE_DIR}/src)

# ── KronMotion library ──────────────────────────────────────────────────────
add_library(kronmotion STATIC
    ${MOTION_CORE_DIR}/TrajectoryGenerator.c
    kron_pi.c
    kron_axis.c
    kronmotion.c
)

target_include_directories(kronmotion PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${MOTION_CORE_DIR}
)

# -fno-math-errno drops the errno branch around every sqrt in the core; no
# result changes, unlike -ffast-math, which would reassociate the formulas
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(kronmotion PRIVATE -fno-math-errno -Wall -Wextra)
endif()

if(UNIX)
    target_link_libraries(kronmotion PUBLIC m)
endif()

# ── Self test ───────────────────────────────────────────────────────────────
if(KRONMOTION_BUILD_TESTS)
    enable_testing()
    add_executable(kronmotion_test test/kronmotion_test.c)
    target_link_libraries(kronmotion_test PRIVATE kronmotion)
    add_test(NAME kronmotion_test COMMAND kronmotion_test)
endif()

# ── GUI tool ────────────────────────────────────────────────────────────────
# Off by default: it was written against the old NC engine and has not been
# ported to the new axis engine yet.
if(KRONMOTION_BUILD_TOOL)
    message(FATAL_ERROR
        "The GUI tool has not been ported to the new axis engine yet. "
        "Leave KRONMOTION_BUILD_TOOL off.")
endif()
