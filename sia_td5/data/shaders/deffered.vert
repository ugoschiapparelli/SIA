#version 410 core

in vec3 vtx_position;
in vec3 vtx_color;
in vec3 vtx_normal;


void main()
{
    gl_Position = vec4(vtx_position,1.0);
}
