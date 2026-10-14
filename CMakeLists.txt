cmake_minimum_required(VERSION 3.20)

project(boids_screensaver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
option(BOIDS_SCREENSAVER_STATIC_RUNTIME "Link MinGW runtime libraries statically" ON)
option(BOIDS_SCREENSAVER_ENABLE_OPENMP "Enable optional OpenMP force calculation" ON)

if (BOIDS_SCREENSAVER_ENABLE_OPENMP)
    find_package(OpenMP)
endif ()

add_library(flock_core STATIC
    src/flock_simulation.cpp
    src/flock_simulation.h
)
target_include_directories(flock_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")

if (BOIDS_SCREENSAVER_ENABLE_OPENMP AND OpenMP_CXX_FOUND)
    target_link_libraries(flock_core PUBLIC OpenMP::OpenMP_CXX)
endif ()

if (MSVC)
    target_compile_options(flock_core PRIVATE /W4 /permissive-)
else ()
    target_compile_options(flock_core PRIVATE -Wall -Wextra -Wpedantic)
endif ()

if (WIN32)
    add_executable(boids_screensaver WIN32
        src/boids_screensaver.cpp
    )
    target_link_libraries(boids_screensaver PRIVATE flock_core opengl32 user32 gdi32)

    if (MSVC)
        target_compile_options(boids_screensaver PRIVATE /W4 /permissive-)
    else ()
        target_compile_options(boids_screensaver PRIVATE -Wall -Wextra -Wpedantic)
        target_link_options(boids_screensaver PRIVATE -mwindows)
        if (BOIDS_SCREENSAVER_STATIC_RUNTIME)
            target_link_options(boids_screensaver PRIVATE -static -static-libgcc -static-libstdc++)
        endif ()
    endif ()

    set_target_properties(boids_screensaver PROPERTIES
        OUTPUT_NAME "boids"
        SUFFIX ".scr"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/bin"
    )
endif ()

include(CTest)
if (BUILD_TESTING)
    add_executable(flock_simulation_tests tests/flock_simulation_tests.cpp)
    target_link_libraries(flock_simulation_tests PRIVATE flock_core)

    if (MSVC)
        target_compile_options(flock_simulation_tests PRIVATE /W4 /permissive-)
    else ()
        target_compile_options(flock_simulation_tests PRIVATE -Wall -Wextra -Wpedantic)
    endif ()

    add_test(NAME flock_simulation_tests COMMAND flock_simulation_tests)
endif ()
