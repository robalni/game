#version 330 core

in vec3 norm_pass;
in float light_pass;

out vec4 color_out;

uniform vec4 color;

void main() {
    color_out = vec4(color.rgb * light_pass, color.a);
}
