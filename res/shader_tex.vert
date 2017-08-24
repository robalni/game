#version 330 core

layout (location = 0) in vec3 pos;
layout (location = 1) in vec2 tex;
layout (location = 2) in vec3 norm;

out vec3 norm_pass;
out float light_pass;
out vec2 tex_pass;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

void main() {
    vec3 light_vec = normalize(vec3(6, -3, 9));
    light_pass = atan(dot(norm, light_vec)/length(norm)*3) * 0.4 + 0.5;
    gl_Position = proj * view * model * vec4(pos, 1);
    norm_pass = norm;
    tex_pass = tex;
}
