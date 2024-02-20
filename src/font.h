// FOR DETAILED CODE EXPLANATION, VISIT THE TUTORIAL SERIES https://www.youtube.com/watch?v=yRYHly3bl2Q&list=PLStQc0GqppuWBDuNWnkQ8rzmyx35AINyt
// --------------------------------------------------------------------------------------------------------------------------------------------

#pragma once

#include <cstdio>
#include <GL/glew.h>

//#include "defines.h"

#define STB_TRUETYPE_IMPLEMENTATION
#include "../lib/stb_truetype.h"
#include "HelperFunctions.h"
#include "GlobalVariables.h"

float fontMinMaxValues[4];
int fontCenter[2];

struct FontVertex {
	glm::vec2 position;
	glm::vec2 texCoords;
	glm::vec4 texColor;
};

struct Font {
	~Font() {
		if (fontVertexBufferData) {
			delete[] fontVertexBufferData;
		}
	}

	void initFont(const char* filename, float fontSize) {
		uint8_t ttfBuffer[96932];
		uint8_t tmpBitmap[512 * 512];

		FILE* file = fopen(filename, "rb");
		if (file == nullptr) {
			std::cout << "Error: file " << filename << " not found!" << std::endl;
			exit(0);
		}
		fseek(file, 0, SEEK_END);
		size_t filesize = ftell(file);
		rewind(file);

		fread(ttfBuffer, 1, filesize, file);
		if (filesize != 96932) {
			std::cout << "Filesize missmatch!" << std::endl;
		}
		stbtt_BakeFontBitmap(ttfBuffer, 0, fontSize, tmpBitmap, 512, 512, 32, 96, cdata);

		glGenTextures(1, &fontTexture);
		glBindTexture(GL_TEXTURE_2D, fontTexture);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, 512, 512, 0, GL_ALPHA, GL_UNSIGNED_BYTE, tmpBitmap);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glBindTexture(GL_TEXTURE_2D, 0);

		glGenVertexArrays(1, &fontVao);
		glBindVertexArray(fontVao);
		glGenBuffers(1, &fontVertexBufferID);
		glBindBuffer(GL_ARRAY_BUFFER, fontVertexBufferID);
		fontVertexBufferCapacity = 20;
		fontVertexBufferData = new FontVertex[fontVertexBufferCapacity * 6];
		glBufferData(GL_ARRAY_BUFFER, sizeof(FontVertex) * 6 * fontVertexBufferCapacity, 0, GL_DYNAMIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(FontVertex), 0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(FontVertex), (const void*)offsetof(FontVertex, texCoords));
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(FontVertex), (const void*)offsetof(FontVertex, texColor));
		glBindVertexArray(0);
	}

	void getStringPixelOffset(const char* text, float x = 0, float y = 0) {
		fontMinMaxValues[0] = 10000.0f;
		fontMinMaxValues[1] = 10000.0f;
		fontMinMaxValues[2] = -10000.0f;
		fontMinMaxValues[3] = -10000.0f;

		while (*text) {
			if (*text >= 32 && *text < 128) {
				stbtt_aligned_quad q;
				stbtt_GetBakedQuad(cdata, 512, 512, *text - 32, &x, &y, &q, 1);

				if (q.x0 < fontMinMaxValues[0]) fontMinMaxValues[0] = q.x0;
				if (q.y0 < fontMinMaxValues[1]) fontMinMaxValues[1] = q.y0;
				if (q.x1 > fontMinMaxValues[2]) fontMinMaxValues[2] = q.x1;
				if (q.y1 > fontMinMaxValues[3]) fontMinMaxValues[3] = q.y1;
			}
			++text;
		}

		fontCenter[0] = (fontMinMaxValues[2] + fontMinMaxValues[0]) / 2.0f;
		fontCenter[1] = (fontMinMaxValues[3] + fontMinMaxValues[1]) / 2.0f;
	}

	void drawString(float x, float y, const char* text, float r, float g, float b, Shader* fontShader) {
		glBindVertexArray(fontVao);
		glBindBuffer(GL_ARRAY_BUFFER, fontVertexBufferID);

		uint32_t len = strlen(text);
		if (fontVertexBufferCapacity < len) {
			fontVertexBufferCapacity = len;
			glBufferData(GL_ARRAY_BUFFER, sizeof(FontVertex) * 6 * fontVertexBufferCapacity, 0, GL_DYNAMIC_DRAW);
			delete[] fontVertexBufferData;
			fontVertexBufferData = new FontVertex[fontVertexBufferCapacity * 6];
		}

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, fontTexture);
		glUniform1i(glGetUniformLocation(fontShader->getShaderId(), "u_texture"), 0);

		FontVertex* vData = fontVertexBufferData;
		uint32_t numVertices = 0;
		while (*text) {
			if (*text >= 32 && *text < 128) {
				stbtt_aligned_quad q;
				stbtt_GetBakedQuad(cdata, 512, 512, *text - 32, &x, &y, &q, 1);

				vData[0].position = glm::vec2(q.x0, q.y0); vData[0].texCoords = glm::vec2(q.s0, q.t0); vData[0].texColor = glm::vec4(r, g, b, 1.0);
				vData[1].position = glm::vec2(q.x1, q.y0); vData[1].texCoords = glm::vec2(q.s1, q.t0); vData[1].texColor = glm::vec4(r, g, b, 1.0);
				vData[2].position = glm::vec2(q.x1, q.y1); vData[2].texCoords = glm::vec2(q.s1, q.t1); vData[2].texColor = glm::vec4(r, g, b, 1.0);
				vData[3].position = glm::vec2(q.x0, q.y1); vData[3].texCoords = glm::vec2(q.s0, q.t1); vData[3].texColor = glm::vec4(r, g, b, 1.0);
				vData[4].position = glm::vec2(q.x0, q.y0); vData[4].texCoords = glm::vec2(q.s0, q.t0); vData[4].texColor = glm::vec4(r, g, b, 1.0);
				vData[5].position = glm::vec2(q.x1, q.y1); vData[5].texCoords = glm::vec2(q.s1, q.t1); vData[5].texColor = glm::vec4(r, g, b, 1.0);

				vData += 6;
				numVertices += 6;
			}
			++text;
		}

		glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(FontVertex) * numVertices, fontVertexBufferData);
		glDrawArrays(GL_TRIANGLES, 0, numVertices);
	}


	static void shader_handle_text(float fps, float pausedPixelOffsetsX,
			float pausedPixelOffsetsY, HelperFunctions *hf, Font &font,
			Shader &fontShader, Font &fontUI, Font &fontBig) {
		GlobalVariables *gv = GlobalVariables::get_instance();
		gv->guiString = "FPS: ";
		gv->guiString.append(hf->to_string_with_precision(fps, 1));
		font.drawString(20.0f, 30.0f, gv->guiString.c_str(), 0.0f, 0.0f, 0.0f,
				&fontShader);
		if (gv->collision) {
			gv->guiString = "Collision";
			font.drawString(500.0f, 30.f, gv->guiString.c_str(), 0.0f, 0.0f, 0.0f,
					&fontShader);
		}
		if (gv->synMode == -1) {
			gv->guiString = "Switch to training or env. initialization";
			font.drawString(200.0f, 30.f, gv->guiString.c_str(), 0.0f, 0.0f, 0.0f,
					&fontShader);
			hf->calcDisplayPos(0, 0, 15.5f);
			gv->guiString = "Kuka youBot";
			if (gv->displayPos[3] > 0.1f) {
				fontUI.getStringPixelOffset(gv->guiString.c_str());
				fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
						-gv->displayPos[1] * 400 + 400 - fontCenter[1],
						gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
			}
			hf->calcDisplayPos(10, 30.4f, 3.5f);
			gv->guiString = "Monitoring";
			if (gv->displayPos[3] > 0.1f) {
				fontUI.getStringPixelOffset(gv->guiString.c_str());
				fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
						-gv->displayPos[1] * 400 + 400 - 12 - fontCenter[1],
						gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
			}
			gv->guiString = "Tool 1";
			if (gv->displayPos[3] > 0.1f) {
				fontUI.getStringPixelOffset(gv->guiString.c_str());
				fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
						-gv->displayPos[1] * 400 + 400 + 12 - fontCenter[1],
						gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
			}
			hf->calcDisplayPos(-23, 3, 3.5f);
			gv->guiString = "Monitoring";
			if (gv->displayPos[3] > 0.1f) {
				fontUI.getStringPixelOffset(gv->guiString.c_str());
				fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
						-gv->displayPos[1] * 400 + 400 - 12 - fontCenter[1],
						gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
			}
			gv->guiString = "Tool 2";
			if (gv->displayPos[3] > 0.1f) {
				fontUI.getStringPixelOffset(gv->guiString.c_str());
				fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
						-gv->displayPos[1] * 400 + 400 + 12 - fontCenter[1],
						gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
			}
			hf->calcDisplayPos(-23, 7.5f, 6.1375f);
			gv->guiString = "camera viewfield";
			if (gv->displayPos[3] > 0.1f) {
				fontUI.getStringPixelOffset(gv->guiString.c_str());
				fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
						-gv->displayPos[1] * 400 + 400 - fontCenter[1],
						gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
			}
			hf->calcDisplayPos(-23, 30, 6);
			gv->guiString = "detected &";
			if (gv->displayPos[3] > 0.1f) {
				fontUI.getStringPixelOffset(gv->guiString.c_str());
				fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
						-gv->displayPos[1] * 400 + 400 - 24 - fontCenter[1],
						gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
			}
			gv->guiString = "localized";
			if (gv->displayPos[3] > 0.1f) {
				fontUI.getStringPixelOffset(gv->guiString.c_str());
				fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
						-gv->displayPos[1] * 400 + 400 - fontCenter[1],
						gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
			}
			gv->guiString = "obstacle";
			if (gv->displayPos[3] > 0.1f) {
				fontUI.getStringPixelOffset(gv->guiString.c_str());
				fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
						-gv->displayPos[1] * 400 + 400 + 24 - fontCenter[1],
						gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
			}
			hf->calcDisplayPos(25, 8, 4);
			gv->guiString = "undetected";
			if (gv->displayPos[3] > 0.1f) {
				fontUI.getStringPixelOffset(gv->guiString.c_str());
				fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
						-gv->displayPos[1] * 400 + 400 - 12 - fontCenter[1],
						gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
			}
			gv->guiString = "obstacle";
			if (gv->displayPos[3] > 0.1f) {
				fontUI.getStringPixelOffset(gv->guiString.c_str());
				fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
						-gv->displayPos[1] * 400 + 400 + 12 - fontCenter[1],
						gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
			}
			hf->calcDisplayPos(8, 21.5, 1.5);
			gv->guiString = "workpiece";
			if (gv->displayPos[3] > 0.1f) {
				fontUI.getStringPixelOffset(gv->guiString.c_str());
				fontUI.drawString(gv->displayPos[0] * 400 + 400 - fontCenter[0],
						-gv->displayPos[1] * 400 + 400 - fontCenter[1],
						gv->guiString.c_str(), 0.0f, 0.0f, 0.0f, &fontShader);
			}
		} else if (gv->synMode == 0) {
			if (gv->randomAgent == -1) {
				gv->guiString = "Training: none";
			} else if (gv->randomAgent < 4) {
				gv->guiString = "Training: loc";
			} else if (gv->randomAgent < 11) {
				gv->guiString = "Training: gv->path";
			} else {
				gv->guiString = "Training: obs";
			}

			font.drawString(200.0f, 30.f, gv->guiString.c_str(), 0.0f, 0.0f, 0.0f,
					&fontShader);
		} else if (gv->synMode == 1) {
			gv->guiString = "Env. initialization";
			font.drawString(200.0f, 30.f, gv->guiString.c_str(), 0.0f, 0.0f, 0.0f,
					&fontShader);
		} else if (gv->synMode == 2) {
			gv->guiString = "Transport simulation";
			font.drawString(200.0f, 30.f, gv->guiString.c_str(), 0.0f, 0.0f, 0.0f,
					&fontShader);
		} else if (gv->synMode == 3) {
			gv->guiString = "Realworld execution";
			font.drawString(200.0f, 30.f, gv->guiString.c_str(), 0.0f, 0.0f, 0.0f,
					&fontShader);
		} else if (gv->synMode == 4) {
			gv->guiString = "Manual control";
			font.drawString(200.0f, 30.f, gv->guiString.c_str(), 0.0f, 0.0f, 0.0f,
					&fontShader);
		} else {
			gv->guiString = "Mode error";
			font.drawString(200.0f, 30.0f, gv->guiString.c_str(), 0.0f, 0.0f, 0.0f,
					&fontShader);
		}

		if (gv->selectionMarkerMode) {
			gv->guiString = "Rot. value: ";
			gv->guiString.append(
					hf->to_string_with_precision(gv->rotationValue, 1));
			gv->guiString.append(" deg");
			font.drawString(500.0f, 30.f, gv->guiString.c_str(), 0.0f, 0.0f, 0.0f,
					&fontShader);
		}
		if (gv->paused) {
			gv->guiString = "PAUSED";
			fontBig.drawString(400.0f - pausedPixelOffsetsX,
					400.0f - pausedPixelOffsetsY, gv->guiString.c_str(), 0.0f, 0.0f,
					0.0f, &fontShader);
		}
	}

private:
	stbtt_bakedchar cdata[96];
	GLuint fontTexture;
	GLuint fontVao;
	GLuint fontVertexBufferID;
	FontVertex* fontVertexBufferData = 0;
	uint32_t fontVertexBufferCapacity;
};
