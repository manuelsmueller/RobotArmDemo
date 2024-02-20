// FOR DETAILED CODE EXPLANATION, VISIT THE TUTORIAL SERIES https://www.youtube.com/watch?v=yRYHly3bl2Q&list=PLStQc0GqppuWBDuNWnkQ8rzmyx35AINyt
// --------------------------------------------------------------------------------------------------------------------------------------------

#version 120

attribute vec2 a_position;
attribute vec2 a_tex_coord;
attribute vec4 a_tex_color;

varying vec2 v_tex_coord;
varying vec4 v_tex_color;

uniform mat4 u_modelViewProj;

void main()
{
    gl_Position = u_modelViewProj * vec4(a_position, 0.5f, 1.0f);
    v_tex_coord = a_tex_coord;
    v_tex_color = a_tex_color;
}