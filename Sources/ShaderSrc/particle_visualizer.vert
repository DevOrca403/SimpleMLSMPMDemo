R"(

#version 430 core

layout (location = 0) in vec2 offset;
layout (location = 1) in float value;

out vec4 vs_color;

void main(void) {
    gl_Position = vec4(offset, 0.0f, 1.0f);
    vs_color = vec4(value, 1.0f - value, 0.0f, 1.0f);
}

)"