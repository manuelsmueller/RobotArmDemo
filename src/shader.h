// FOR DETAILED CODE EXPLANATION, VISIT THE TUTORIAL SERIES https://www.youtube.com/watch?v=yRYHly3bl2Q&list=PLStQc0GqppuWBDuNWnkQ8rzmyx35AINyt
// --------------------------------------------------------------------------------------------------------------------------------------------

#pragma once
#include <GL/glew.h>
#include <string>
//#include "defines.h"


//#include "font.h"

struct Shader {
public:
	Shader(const char* vertexShaderFilename, const char* fragmentDhaderFilename);
	virtual ~Shader();

	void bind();
	void unbind();

	GLuint getShaderId() {
		return shaderId;
	}
//	static void shader_handle_text(float fps, float pausedPixelOffsetsX,
//			float pausedPixelOffsetsY, HelperFunctions *hf, Font &font,
//			Shader &fontShader, Font &fontUI, Font &fontBig);

private:
	GLuint compile(std::string shaderSource, GLenum type);
	std::string parse(const char* filename);
	GLuint createShader(const char* vertexShaderFilename, const char* fragmentShaderFilename);

	GLuint shaderId;
};
