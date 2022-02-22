cmake_minimum_required(VERSION 3.21)
project(OrcaMLSMPMDemo)

set(CMAKE_CXX_STANDARD 17)

set(ROOT ${PROJECT_SOURCE_DIR})

# glfw
# disable glfw test build.
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

add_subdirectory(Externals/glfw)

add_executable (OrcaMLSMPMDemo)

find_package (Eigen3 3.3 NO_MODULE)

if(TARGET Eigen3::Eigen)
    target_link_libraries (OrcaMLSMPMDemo
            PRIVATE
            Eigen3::Eigen
            )
elseif(EXISTS ${ROOT}/Externals/eigen/signature_of_eigen3_matrix_library)
    target_include_directories (OrcaMLSMPMDemo
            PRIVATE
            ${ROOT}/Externals/eigen
    )
else()
    message( FATAL_ERROR "Failed to find Eigen3")
endif()

target_sources (OrcaMLSMPMDemo
        PRIVATE
        Externals/glad/include/glad/glad.h
        Externals/glad/src/glad.c
        Sources/main.cpp
        Sources/particle_visualizer.cpp
        Sources/particle_visualizer.h
        Sources/grid.cpp
        Sources/grid.h Sources/function.h
        Sources/particle_update.h
        Sources/pg_transfer.h
        Sources/particle_system.cpp
        Sources/particle_system.h)

target_link_libraries (OrcaMLSMPMDemo
        glfw)

target_include_directories (OrcaMLSMPMDemo
        PRIVATE
        Externals/glad/include
        Externals/glfw/include)
