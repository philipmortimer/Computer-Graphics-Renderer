#include <CanvasTriangle.h>
#include <DrawingWindow.h>
#include <Utils.h>
#include <fstream>
#include <vector>
#include <glm/glm.hpp>
#include <CanvasPoint.h>
#include <Colour.h>
#include <CanvasTriangle.h>
#include <TextureMap.h>
#include <TexturePoint.h>

#define WIDTH 320
#define HEIGHT 240

void drawLine(DrawingWindow &window, CanvasPoint from, CanvasPoint to, Colour colour) {
	float xDiff = to.x - from.x;
	float yDiff = to.y - from.y;
	float numberOfSteps = abs(xDiff) > abs(yDiff)? abs(xDiff) : abs(yDiff);
	// Handles case where drawLine is called from a point to that same point by drawing that point.
	if (numberOfSteps == 0) numberOfSteps = 1;
	float xStepSize = xDiff / numberOfSteps;
	float yStepSize = yDiff / numberOfSteps;
	uint32_t col = (255 << 24) + (colour.red << 16) + (colour.green << 8) + colour.blue;
	for (float i = 0.0; i <= numberOfSteps; i++) {
		float x = from.x + (xStepSize * i);
		float y = from.y + (yStepSize * i);
		window.setPixelColour(round(x), round(y), col);
	}
}

void testDrawLine(DrawingWindow& window) {
	// A line from the top-left corner of the window to the centre of the window
	drawLine(window, CanvasPoint(0, 0), CanvasPoint(WIDTH / 2, HEIGHT / 2), Colour(255, 255, 255));
	// A line from the top-right corner of the window to the centre of the window
	drawLine(window, CanvasPoint(WIDTH - 1, 0), CanvasPoint(WIDTH / 2, HEIGHT / 2), Colour(255, 255, 255));
	// A vertical line all the way down the middle of the screen
	drawLine(window, CanvasPoint(WIDTH / 2, 0), CanvasPoint(WIDTH / 2, HEIGHT - 1), Colour(255, 255, 255));
	// A horizontal line a third the width of the screen, centred both horizontallyand vertically
	drawLine(window, CanvasPoint(WIDTH / 3, HEIGHT  / 2), CanvasPoint(2 * WIDTH / 3, HEIGHT / 2), Colour(255, 255, 255));
}

void drawStrokedTriangle(DrawingWindow& window, CanvasTriangle triangle, Colour colour) {
	drawLine(window, triangle.vertices[0], triangle.vertices[1], colour);
	drawLine(window, triangle.vertices[1], triangle.vertices[2], colour);
	drawLine(window, triangle.vertices[2], triangle.vertices[0], colour);
}

CanvasTriangle randTriangle() {
	return CanvasTriangle(CanvasPoint(rand() % WIDTH, rand() % HEIGHT), CanvasPoint(rand() % WIDTH, rand() % HEIGHT),
		CanvasPoint(rand() % WIDTH, rand() % HEIGHT));
}

Colour randColour() {
	return Colour(rand() % 256, rand() % 256, rand() % 256);
}

std::pair<CanvasTriangle, CanvasTriangle> getTwoFlatTriangles(CanvasTriangle triangle) {
	std::pair<CanvasTriangle, CanvasTriangle> flatTris;
	// Sorts vertices with top most elements (lowest y value) at start of array. Using unrolled bubble sort
	if (triangle.vertices[0].y > triangle.vertices[1].y) std::swap(triangle.vertices[0], triangle.vertices[1]);
	if (triangle.vertices[1].y > triangle.vertices[2].y) std::swap(triangle.vertices[1], triangle.vertices[2]);
	if (triangle.vertices[0].y > triangle.vertices[1].y) std::swap(triangle.vertices[0], triangle.vertices[1]);

	CanvasPoint top = triangle.vertices[0];
	CanvasPoint bottom = triangle.vertices[2];
	CanvasPoint mid = triangle.vertices[1];
	// Calculates point that makes non flat bottomed triangle into two flat bottomed ones
	float y = mid.y;
	float xToYRatio = (bottom.x - top.x) / (bottom.y - top.y);
	float x = top.x + (xToYRatio * (y - top.y));
	CanvasPoint midArt = CanvasPoint(x, y);
	// Assigns left / right points to triangle
	CanvasPoint left = midArt;
	CanvasPoint right = mid;
	if (midArt.x > mid.x) {
		left = mid;
		right = midArt;
	}
	// Note this return order is predefined and used in code. Better practice would be to make custom return class to prevent errors.
	flatTris.first = CanvasTriangle(CanvasPoint(top), CanvasPoint(left), CanvasPoint(right));
	flatTris.second = CanvasTriangle(CanvasPoint(bottom), CanvasPoint(left), CanvasPoint(right));
	return flatTris;
}

void drawTopFlatTriangle(DrawingWindow& window, CanvasTriangle triangle, Colour colour) {
	CanvasPoint left = triangle.vertices[1];
	CanvasPoint right = triangle.vertices[2];
	CanvasPoint top = triangle.vertices[0];
	// Draws triangle
	float changeXPerRowLeft = (left.x - top.x) / (left.y - top.y);
	float changeXPerRowRight = (right.x - top.x) / (right.y - top.y);
	float xLeft = top.x;
	float xRight = top.x;
	for (int row = round(top.y); row <= round(left.y); row++) {
		drawLine(window, CanvasPoint(round(xLeft), row), CanvasPoint(round(xRight), row), colour);
		xLeft += changeXPerRowLeft;
		xRight += changeXPerRowRight;
	}
}

void drawBottomFlatTriangle(DrawingWindow& window, CanvasTriangle triangle, Colour colour) {
	CanvasPoint left = triangle.vertices[1];
	CanvasPoint right = triangle.vertices[2];
	CanvasPoint bottom = triangle.vertices[0];
	// Draws triangle
	float changeXPerRowLeft = (left.x - bottom.x) / (left.y - bottom.y);
	float changeXPerRowRight = (right.x - bottom.x) / (right.y - bottom.y);
	float xLeft = left.x;
	float xRight = right.x;
	for (int row = round(left.y); row <= round(bottom.y); row++) {
		drawLine(window, CanvasPoint(round(xLeft), row), CanvasPoint(round(xRight), row), colour);
		xLeft += changeXPerRowLeft;
		xRight += changeXPerRowRight;
	}
}

void drawFilledTriangle(DrawingWindow& window, CanvasTriangle triangle, Colour colour) {
	std::pair<CanvasTriangle, CanvasTriangle> tris = getTwoFlatTriangles(triangle);
	drawTopFlatTriangle(window, tris.first, colour);
	drawBottomFlatTriangle(window, tris.second, colour);
}

void drawTexturedTriangle(DrawingWindow& window, CanvasTriangle triangle, TextureMap texture) {
	std::pair<CanvasTriangle, CanvasTriangle> tris = getTwoFlatTriangles(triangle);
	CanvasPoint left = tris.first.vertices[1];
	CanvasPoint right = tris.first.vertices[2];
	CanvasPoint bottom = tris.second.vertices[0];
	CanvasPoint top = tris.first.vertices[0];
	// Either left or right variable is artificially created and thus does not have a texturepoint associated with it.
	// Checks for this and creates the corresponding texturepoint. In code, this is signified by the x and y coords
	// of the texture being (-1, -1). This code corrects the texturepoint.
	// TODO IMPLEMENT THIS
	bool isLeftArtificial = false;
	// Stores points from original triangle which are left / right
	CanvasPoint leftOriginal = isLeftArtificial ? bottom : left;
	CanvasPoint rightOriginal = isLeftArtificial ? right : bottom;
	// Draws top triangle
	float changeXPerRowLeft = (left.x - top.x) / (left.y - top.y);
	float changeXPerRowRight = (right.x - top.x) / (right.y - top.y);
	float xLeft = top.x;
	float xRight = top.x;
	for (int row = round(top.y); row <= round(left.y); row++) {
		// Calculates number of pixels to sample from texture
		int noPixels = round(xRight - xLeft);
		noPixels = noPixels == 0 ? 1 : noPixels; // Ensures at least one point is sampled per row
		// Calculates what percent of way down x and y it is on original triangle - left
		float topToLeftPercentX = abs((xLeft - top.x) / (leftOriginal.x - top.x));
		float topToLeftPercentY = abs((row - top.y) / (leftOriginal.y - top.y));
		// Calculates what percent of way down x and y it is on original triangle - right
		float topToRightPercentX = abs((xRight - top.x) / (rightOriginal.x - top.x));
		float topToRightPercentY = abs((row - top.y) / (rightOriginal.y - top.y));
		// Calculates corresponding texture mapping (as it will also be same percentage down on those points)
		float xTextureLeft = top.texturePoint.x + (topToLeftPercentX * (leftOriginal.texturePoint.x - top.texturePoint.x));
		float yTextureLeft = top.texturePoint.y + (topToLeftPercentY * (leftOriginal.texturePoint.y - top.texturePoint.y));
		float xTextureRight = top.texturePoint.x + (topToRightPercentX * (rightOriginal.texturePoint.x - top.texturePoint.x));
		float yTextureRight = top.texturePoint.y + (topToRightPercentY * (rightOriginal.texturePoint.y - top.texturePoint.y));
		// Calculates line equation defined by points
		float xDiff = xTextureRight - xTextureLeft;
		float yDiff = yTextureRight - yTextureLeft;
		float xStepSize = xDiff / noPixels;
		float yStepSize = yDiff / noPixels;
		// Samples each pixel from texture map and draws it
		for (float pix = 0.0; pix <= noPixels; pix++) {
			float xText = xTextureLeft + (xStepSize * pix);
			float yText = yTextureLeft + (yStepSize * pix);
			window.setPixelColour(round(pix) + xLeft, row, texture.pixels[(yText * texture.height) + xText]);
		}
		xLeft += changeXPerRowLeft;
		xRight += changeXPerRowRight;
	}


}

void testTexturedTriangle(DrawingWindow& window) {
	// Test points
	CanvasPoint p1 = CanvasPoint(160, 10);
	p1.texturePoint = TexturePoint(195, 5);
	CanvasPoint p2 = CanvasPoint(300, 230);
	p2.texturePoint = TexturePoint(395, 380);	
	CanvasPoint p3 = CanvasPoint(10, 150);
	p3.texturePoint = TexturePoint(65, 330);
	// Texture map
	TextureMap mp = TextureMap("texture.ppm");
	// Creates triangle
	drawTexturedTriangle(window, CanvasTriangle(p1, p2, p3), mp);
	drawStrokedTriangle(window, CanvasTriangle(p1, p2, p3), Colour(255, 255, 255));
}

void handleEvent(SDL_Event event, DrawingWindow &window) {
	if (event.type == SDL_KEYDOWN) {
		if (event.key.keysym.sym == SDLK_LEFT) std::cout << "LEFT" << std::endl;
		else if (event.key.keysym.sym == SDLK_RIGHT) std::cout << "RIGHT" << std::endl;
		else if (event.key.keysym.sym == SDLK_UP) std::cout << "UP" << std::endl;
		else if (event.key.keysym.sym == SDLK_DOWN) std::cout << "DOWN" << std::endl;
		else if (event.key.keysym.sym == SDLK_u) drawStrokedTriangle(window, randTriangle(), randColour());
		else if (event.key.keysym.sym == SDLK_f) {
			CanvasTriangle tri = randTriangle();
			drawFilledTriangle(window, tri, randColour());
			drawStrokedTriangle(window, tri, Colour(255, 255, 255));
		}
	} else if (event.type == SDL_MOUSEBUTTONDOWN) {
		window.savePPM("output.ppm");
		window.saveBMP("output.bmp");
	}
}


int main(int argc, char *argv[]) {
	DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
	SDL_Event event;
	while (true) {
		// We MUST poll for events - otherwise the window will freeze !
		if (window.pollForInputEvents(event)) handleEvent(event, window);
		testTexturedTriangle(window);
		// Need to render the frame at the end, or nothing actually gets shown on the screen !
		window.renderFrame();
	}
}
