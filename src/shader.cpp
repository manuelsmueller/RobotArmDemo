// FOR DETAILED CODE EXPLANATION, VISIT THE TUTORIAL SERIES https://www.youtube.com/watch?v=yRYHly3bl2Q&list=PLStQc0GqppuWBDuNWnkQ8rzmyx35AINyt
// --------------------------------------------------------------------------------------------------------------------------------------------

#include "shader.h"
#include <fstream>
#include <iostream>

#pragma warning(disable:4996)

Shader::Shader(const char* vertexShaderFilename, const char* fragmentShaderFilename) {
	shaderId = createShader(vertexShaderFilename, fragmentShaderFilename);
}

Shader::~Shader() {
	glDeleteProgram(shaderId);
}

void Shader::bind() {
	glUseProgram(shaderId);
}

void Shader::unbind() {
	glUseProgram(0);
}

GLuint Shader::compile(std::string shaderSource, GLenum type) {
	GLuint id = glCreateShader(type);
	const char* src = shaderSource.c_str();
	glShaderSource(id, 1, &src, 0);
	glCompileShader(id);

	int result;
	glGetShaderiv(id, GL_COMPILE_STATUS, &result);
	if (result != GL_TRUE) {
		int length = 0;
		glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
		char* message = new char[length];
		glGetShaderInfoLog(id, length, &length, message);
		std::cout << "Shader compilation error: " << message << std::endl;
		delete[] message;
		return 0;
	}
	return id;
}

std::string Shader::parse(const char* filename) {
	FILE* file;
	file = fopen(filename, "rb");
	if (file == nullptr) {
		std::cout << "File " << filename << " not found" << std::endl;
		return 0;
	}

	std::string contents;
	fseek(file, 0, SEEK_END);
	size_t filesize = ftell(file);
	rewind(file);
	contents.resize(filesize);

	fread(&contents[0], 1, filesize, file);
	fclose(file);

	return contents;
}

GLuint Shader::createShader(const char* vertexShaderFilename, const char* fragmentShaderFilename) {
	std::string vertexShaderSource = parse(vertexShaderFilename);
	std::string fragmentShaderSource = parse(fragmentShaderFilename);

	GLuint program = glCreateProgram();
	GLuint vs = compile(vertexShaderSource, GL_VERTEX_SHADER);
	GLuint fs = compile(fragmentShaderSource, GL_FRAGMENT_SHADER);

	glAttachShader(program, vs);
	glAttachShader(program, fs);
	glLinkProgram(program);

#ifdef _RELEASE
	glDetachShader(program, vs);
	glDetachShader(program, fs);

	glDeleteShader(vs);
	glDeleteShader(fs);
#endif

	return program;
}
//
//void Shader::shader_handle_text(float fps, float pausedPixelOffsetsX,
//		float pausedPixelOffsetsY, HelperFunctions *hf, Font &font,
//		Shader &fontShader, Font &fontUI, Font &fontBig) { GlobalVariables *gv = GlobalVariables::get_instance();
//	gv->guiString = "FPS: ";
//	gv->guiString.append(hf->to_string_with_precision(fps, 1));
//	font.drawString(20.0f, 30.0f, gv->guiString.c_str(), 0.0f, 0.0f, 0.0f,
//			&fontShader);
//	if (gv->collision) {
//		gv->guiString = "Collision";
//		font.drawString(500.0f, 30.f, gv->guiString.c_str(), 0.0f, 0.0f, 0.0f,
//				&fontShader);
//	}
//	if (gv->synMode == -1) {
//		gv->guiString = "Switch to training or env. initialization";
//		font.drawString(200.0f, 30.f, gv->guiString.c_str(), 0.0f, 0.0f, 0.0f,
//				&fontShader);
//		hf->calcDisplayPos(0, 0, 15.5f);
//		gv->guiString = "Kuka youBot";
//		if (gv->displayPos[3] > 0.1f) {
//			fontUI.getStringPixelOffset(gv->guiString.c_str());
//			fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
//					-gv->displayPos[1] * 400 + 400 - fontCenter[1],
//					gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
//		}
//		hf->calcDisplayPos(10, 30.4f, 3.5f);
//		gv->guiString = "Monitoring";
//		if (gv->displayPos[3] > 0.1f) {
//			fontUI.getStringPixelOffset(gv->guiString.c_str());
//			fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
//					-gv->displayPos[1] * 400 + 400 - 12 - fontCenter[1],
//					gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
//		}
//		gv->guiString = "Tool 1";
//		if (gv->displayPos[3] > 0.1f) {
//			fontUI.getStringPixelOffset(gv->guiString.c_str());
//			fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
//					-gv->displayPos[1] * 400 + 400 + 12 - fontCenter[1],
//					gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
//		}
//		hf->calcDisplayPos(-23, 3, 3.5f);
//		gv->guiString = "Monitoring";
//		if (gv->displayPos[3] > 0.1f) {
//			fontUI.getStringPixelOffset(gv->guiString.c_str());
//			fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
//					-gv->displayPos[1] * 400 + 400 - 12 - fontCenter[1],
//					gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
//		}
//		gv->guiString = "Tool 2";
//		if (gv->displayPos[3] > 0.1f) {
//			fontUI.getStringPixelOffset(gv->guiString.c_str());
//			fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
//					-gv->displayPos[1] * 400 + 400 + 12 - fontCenter[1],
//					gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
//		}
//		hf->calcDisplayPos(-23, 7.5f, 6.1375f);
//		gv->guiString = "camera viewfield";
//		if (gv->displayPos[3] > 0.1f) {
//			fontUI.getStringPixelOffset(gv->guiString.c_str());
//			fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
//					-gv->displayPos[1] * 400 + 400 - fontCenter[1],
//					gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
//		}
//		hf->calcDisplayPos(-23, 30, 6);
//		gv->guiString = "detected &";
//		if (gv->displayPos[3] > 0.1f) {
//			fontUI.getStringPixelOffset(gv->guiString.c_str());
//			fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
//					-gv->displayPos[1] * 400 + 400 - 24 - fontCenter[1],
//					gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
//		}
//		gv->guiString = "localized";
//		if (gv->displayPos[3] > 0.1f) {
//			fontUI.getStringPixelOffset(gv->guiString.c_str());
//			fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
//					-gv->displayPos[1] * 400 + 400 - fontCenter[1],
//					gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
//		}
//		gv->guiString = "obstacle";
//		if (gv->displayPos[3] > 0.1f) {
//			fontUI.getStringPixelOffset(gv->guiString.c_str());
//			fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
//					-gv->displayPos[1] * 400 + 400 + 24 - fontCenter[1],
//					gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
//		}
//		hf->calcDisplayPos(25, 8, 4);
//		gv->guiString = "undetected";
//		if (gv->displayPos[3] > 0.1f) {
//			fontUI.getStringPixelOffset(gv->guiString.c_str());
//			fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
//					-gv->displayPos[1] * 400 + 400 - 12 - fontCenter[1],
//					gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
//		}
//		gv->guiString = "obstacle";
//		if (gv->displayPos[3] > 0.1f) {
//			fontUI.getStringPixelOffset(gv->guiString.c_str());
//			fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
//					-gv->displayPos[1] * 400 + 400 + 12 - fontCenter[1],
//					gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
//		}
//		hf->calcDisplayPos(8, 21.5, 1.5);
//		gv->guiString = "workpiece";
//		if (gv->displayPos[3] > 0.1f) {
//			fontUI.getStringPixelOffset(gv->guiString.c_str());
//			fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
//					-gv->displayPos[1] * 400 + 400 - fontCenter[1],
//					gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
//		}
//	} else if (gv->synMode == 0) {
//		if (gv->randomAgent == -1) {
//			gv->guiString = "Training: none";
//		} else if (gv->randomAgent < 4) {
//			gv->guiString = "Training: loc";
//		} else if (gv->randomAgent < 11) {
//			gv->guiString = "Training: gv->path";
//		} else {
//			gv->guiString = "Training: obs";
//		}
//
//		font.drawString(200.0f, 30.f, gv->guiString.c_str(), 0.0f, 0.0f, 0.0f,
//				&fontShader);
//	} else if (gv->synMode == 1) {
//		gv->guiString = "Env. initialization";
//		font.drawString(200.0f, 30.f, gv->guiString.c_str(), 0.0f, 0.0f, 0.0f,
//				&fontShader);
//	} else if (gv->synMode == 2) {
//		gv->guiString = "Transport simulation";
//		font.drawString(200.0f, 30.f, gv->guiString.c_str(), 0.0f, 0.0f, 0.0f,
//				&fontShader);
//	} else if (gv->synMode == 3) {
//		gv->guiString = "Realworld execution";
//		font.drawString(200.0f, 30.f, gv->guiString.c_str(), 0.0f, 0.0f, 0.0f,
//				&fontShader);
//	} else if (gv->synMode == 4) {
//		gv->guiString = "Manual control";
//		font.drawString(200.0f, 30.f, gv->guiString.c_str(), 0.0f, 0.0f, 0.0f,
//				&fontShader);
//	} else {
//		gv->guiString = "Mode error";
//		font.drawString(200.0f, 30.0f, gv->guiString.c_str(), 0.0f, 0.0f, 0.0f,
//				&fontShader);
//	}
//
//	if (gv->selectionMarkerMode) {
//		gv->guiString = "Rot. value: ";
//		gv->guiString.append(
//				hf->to_string_with_precision(gv->rotationValue, 1));
//		gv->guiString.append(" deg");
//		font.drawString(500.0f, 30.f, gv->guiString.c_str(), 0.0f, 0.0f, 0.0f,
//				&fontShader);
//	}
//	if (gv->paused) {
//		gv->guiString = "PAUSED";
//		fontBig.drawString(400.0f - pausedPixelOffsetsX,
//				400.0f - pausedPixelOffsetsY, gv->guiString.c_str(), 0.0f, 0.0f,
//				0.0f, &fontShader);
//	}
//}
