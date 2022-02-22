#ifndef ORCAMPMDEMO_PARTICLE_VISUALIZER_H
#define ORCAMPMDEMO_PARTICLE_VISUALIZER_H

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Dense>

namespace fs = std::filesystem;
using SClock = std::chrono::system_clock;

class particle_visualizer {
public:
    particle_visualizer() = default;
    bool initialize();
    bool load_shader();
    void init_buffer();
    template<class PositionIter, class ValueIter>
    void render(PositionIter _pos_begin, PositionIter _pos_end,
                ValueIter _val_begin);
    bool should_close();
private:
    enum class log_type {
        ERROR = 0,
        WARN = 1,
        INFO = 2
    };
    std::string log_prefix(particle_visualizer::log_type _log_type);
    GLFWwindow* m_window = nullptr;

    GLuint m_shader_program = 0;
    GLuint m_buffers[2] = {0, 0};
    GLuint m_vertex_array = 0;

    SClock::time_point m_running_time_begin;
};

template<class PositionIter, class ValueIter>
void particle_visualizer::render(PositionIter _pos_begin, PositionIter _pos_end, ValueIter _val_begin) {
    int buffer_size = (_pos_end - _pos_begin)/2;
    float* pos_begin_pointer = &(*_pos_begin);
    float* val_begin_pointer = &(*_val_begin);

    glBindVertexArray(m_vertex_array);

    glBindBuffer(GL_ARRAY_BUFFER, m_buffers[0]);
    glBufferData(GL_ARRAY_BUFFER, buffer_size * 2 * static_cast<int>(sizeof(float)), pos_begin_pointer, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, m_buffers[1]);
    glBufferData(GL_ARRAY_BUFFER, buffer_size * static_cast<int>(sizeof(float)), val_begin_pointer, GL_DYNAMIC_DRAW);

    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    glUseProgram(m_shader_program);

    glPointSize(3);
    glDrawArrays(GL_POINTS, 0, buffer_size);
    glfwSwapBuffers(m_window);
    glfwPollEvents();
}

#endif //ORCAMPMDEMO_PARTICLE_VISUALIZER_H
