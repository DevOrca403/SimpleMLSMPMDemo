cmake_minimum_required(VERSION 3.21)
project(OrcaMLSMPMDemo)

set(CMAKE_CXX_STANDARD 17)

# glfw
# disable glfw test build.
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

add_subdirectory(Externals/glfw)

find_package(OpenGL REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable (OrcaMLSMPMDemo
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
        Eigen3::Eigen
        OpenGL::GL
        glfw)

target_include_directories (OrcaMLSMPMDemo
        PRIVATE
        Externals/glad/include
        Externals/glfw/include)
