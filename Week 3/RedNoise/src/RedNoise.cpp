#include <CanvasTriangle.h>
#include <DrawingWindow.h>
#include <Utils.h>
#include <fstream>
#include <vector>
#include <glm/glm.hpp>

#define WIDTH 320
#define HEIGHT 240

void draw(DrawingWindow &window) {
	window.clearPixels();
	for (size_t y = 0; y < window.height; y++) {
		for (size_t x = 0; x < window.width; x++) {
			float red = rand() % 256;
			float green = 0.0;
			float blue = 0.0;
			uint32_t colour = (255 << 24) + (int(red) << 16) + (int(green) << 8) + int(blue);
			window.setPixelColour(x, y, colour);
		}
	}
}

void handleEvent(SDL_Event event, DrawingWindow &window) {
	if (event.type == SDL_KEYDOWN) {
		if (event.key.keysym.sym == SDLK_LEFT) std::cout << "LEFT" << std::endl;
		else if (event.key.keysym.sym == SDLK_RIGHT) std::cout << "RIGHT" << std::endl;
		else if (event.key.keysym.sym == SDLK_UP) std::cout << "UP" << std::endl;
		else if (event.key.keysym.sym == SDLK_DOWN) std::cout << "DOWN" << std::endl;
	} else if (event.type == SDL_MOUSEBUTTONDOWN) {
		window.savePPM("output.ppm");
		window.saveBMP("output.bmp");
	}
}

std::vector<float> interpolateSingleFloats(float from, float to, int numberOfValues) {
	float stepSize = (to - from) / (numberOfValues - 1.0);
	std::vector<float> intVec (numberOfValues);
	intVec[0] = from;
	intVec[numberOfValues - 1] = to;
	for (int i = 1; i < numberOfValues - 1; i++) {
		intVec[i] = intVec[i - 1] + stepSize;
	}
	return intVec;
}

void drawGrayscaleWindow(DrawingWindow& window) {
	window.clearPixels();
	std::vector<float> brightness = interpolateSingleFloats(255.0, 0.0, window.width);
	for (size_t y = 0; y < window.height; y++) {
		for (size_t x = 0; x < window.width; x++) {
			uint32_t colour = (255 << 24) + (int(brightness[x]) << 16) + (int(brightness[x]) << 8) + int(brightness[x]);
			window.setPixelColour(x, y, colour);
		}
	}
}

std::vector<glm::vec3> interpolateThreeElementValues(glm::vec3 from, glm::vec3 to, int numberOfValues) {
	glm::vec3 stepSize = (to - from) * float((1.0 / (numberOfValues - 1.0)));
	std::vector<glm::vec3> intVec(numberOfValues);
	intVec[0] = from;
	intVec[numberOfValues - 1] = to;
	for (int i = 1; i < numberOfValues - 1; i++) {
		intVec[i] = intVec[i - 1] + stepSize;
	}
	return intVec;
}

void drawColourWindow(DrawingWindow& window) {
	glm::vec3 topLeft(255, 0, 0);        // red 
	glm::vec3 topRight(0, 0, 255);       // blue 
	glm::vec3 bottomRight(0, 255, 0);    // green 
	glm::vec3 bottomLeft(255, 255, 0);   // yellow
	std::vector<glm::vec3> leftColInter = interpolateThreeElementValues(topLeft, bottomLeft, window.height);
	std::vector<glm::vec3> rightColInter = interpolateThreeElementValues(topRight, bottomRight, window.height);
	window.clearPixels();
	for (size_t y = 0; y < window.height; y++) {
		std::vector<glm::vec3> rowCol = interpolateThreeElementValues(leftColInter[y], rightColInter[y], window.width);
		for (size_t x = 0; x < window.width; x++) {
			uint32_t colour = (255 << 24) + (int(rowCol[x][0]) << 16) + (int(rowCol[x][1]) << 8) + int(rowCol[x][2]);
			window.setPixelColour(x, y, colour);
		}
	}
}

int main(int argc, char *argv[]) {
	DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
	SDL_Event event;
	while (true) {
		// We MUST poll for events - otherwise the window will freeze !
		if (window.pollForInputEvents(event)) handleEvent(event, window);
		drawColourWindow(window);
		// Need to render the frame at the end, or nothing actually gets shown on the screen !
		window.renderFrame();
	}
}
