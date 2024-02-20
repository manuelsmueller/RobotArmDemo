// FOR DETAILED CODE EXPLANATION, VISIT THE TUTORIAL SERIES https://www.youtube.com/watch?v=yRYHly3bl2Q&list=PLStQc0GqppuWBDuNWnkQ8rzmyx35AINyt
// --------------------------------------------------------------------------------------------------------------------------------------------

#version 330 core

layout(location = 0) out vec4 f_color;

in vec4 v_color;
// uniform vec4 u_color;

void main() {
	f_color = v_color;
	// f_color = vec4(vec3(gl_FragCoord.z), 1.0f);

}