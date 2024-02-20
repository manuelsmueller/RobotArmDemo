// FOR DETAILED CODE EXPLANATION, VISIT THE TUTORIAL SERIES https://www.youtube.com/watch?v=yRYHly3bl2Q&list=PLStQc0GqppuWBDuNWnkQ8rzmyx35AINyt
// --------------------------------------------------------------------------------------------------------------------------------------------
  
#version 120

varying vec2 v_tex_coord;
varying vec4 v_tex_color;

uniform sampler2D u_texture;

void main()
{
    gl_FragColor = vec4(texture2D(u_texture, v_tex_coord).a) * v_tex_color;
}