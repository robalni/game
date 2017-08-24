#version 330 core

in vec3 norm_pass;
in float light_pass;
in vec2 tex_pass;

out vec4 color_out;

uniform vec4 color;
uniform sampler2D samp;

void main() {
    color_out = texture(samp, tex_pass) * color * light_pass;
}
