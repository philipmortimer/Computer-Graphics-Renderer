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
#include <ModelTriangle.h>

#include <unordered_map>

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
	flatTris.first = CanvasTriangle(top, left, right);
	flatTris.second = CanvasTriangle(bottom, left, right);
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

void setCanvasPointTexture(CanvasPoint& pt, CanvasPoint top, CanvasPoint bottom) {
	if (top.y == bottom.y) {
		std::cout << "HERE" << std::endl;
		pt.texturePoint.y = top.texturePoint.y;
	} else {
		pt.texturePoint.y = top.texturePoint.y +
			(((pt.y - top.y) / (bottom.y - top.y)) * (bottom.texturePoint.y - top.texturePoint.y));
	}

	// Prevents division by zero error
	if (top.x == bottom.x) {
		pt.texturePoint.x = top.texturePoint.x;
		std::cout << "HERE" << std::endl;
	} else {
		pt.texturePoint.x = top.texturePoint.x +
			(((pt.x - top.x) / (bottom.x - top.x)) * (bottom.texturePoint.x - top.texturePoint.x));
	}

}

void drawTexturedTriangle(DrawingWindow& window, CanvasTriangle triangle, TextureMap texture) {
	std::pair<CanvasTriangle, CanvasTriangle> tris = getTwoFlatTriangles(triangle);
	CanvasPoint left = tris.first.vertices[1];
	CanvasPoint right = tris.first.vertices[2];
	CanvasPoint bottom = tris.second.vertices[0];
	CanvasPoint top = tris.first.vertices[0];
	// Either left or right variable is artificially created and thus does not have a texturepoint associated with it.
	// Checks for this and creates the corresponding texturepoint.
	bool isLeftArtificial = (right.texturePoint.x == triangle.vertices[0].x && right.texturePoint.y == triangle.vertices[0].y)
		|| (right.texturePoint.x == triangle.vertices[1].x && right.texturePoint.y == triangle.vertices[1].y)
		|| (right.texturePoint.x == triangle.vertices[2].x && right.texturePoint.y == triangle.vertices[2].y);
	if (isLeftArtificial) setCanvasPointTexture(left, top, bottom);
	else setCanvasPointTexture(right, top, bottom);
	// Draws top triangle
	float changeXPerRowLeft = (left.x - top.x) / (left.y - top.y);
	float changeXPerRowRight = (right.x - top.x) / (right.y - top.y);
	float xLeft = top.x;
	float xRight = top.x;
	for (int row = round(top.y); row <= round(left.y); row++) {
		// Calculates number of pixels to sample from texture
		int noPixels = round(xRight - xLeft);
		noPixels = noPixels == 0 ? 1 : noPixels; // Ensures at least one point is sampled per row
		// Calculates points and corresponding texture points
		CanvasPoint leftPt = CanvasPoint(xLeft, row);
		CanvasPoint rightPt = CanvasPoint(xRight, row);
		setCanvasPointTexture(leftPt, top, left);
		setCanvasPointTexture(rightPt, top, right);
		// Calculates line equation defined by points
		float xDiff = rightPt.texturePoint.x - leftPt.texturePoint.x;
		float yDiff = rightPt.texturePoint.y - leftPt.texturePoint.y;
		float xStepSize = xDiff / noPixels;
		float yStepSize = yDiff / noPixels;
		// Samples each pixel from texture map and draws it
		for (float pix = 0.0; round(pix) <= noPixels; pix++) {
			int xText = round(leftPt.texturePoint.x + (xStepSize * pix));
			int yText = round(leftPt.texturePoint.y + (yStepSize * pix));
			window.setPixelColour(round(pix) + xLeft, row, texture.pixels[(yText * texture.width) + xText]);
		}
		xLeft += changeXPerRowLeft;
		xRight += changeXPerRowRight;
	}
	// Draws bottom triangle
	changeXPerRowLeft = (left.x - bottom.x) / (left.y - bottom.y);
	changeXPerRowRight = (right.x - bottom.x) / (right.y - bottom.y);
	xLeft = left.x;
	xRight = right.x;
	for (int row = round(left.y); row <= round(bottom.y); row++) {
		// Calculates number of pixels to sample from texture
		int noPixels = round(xRight - xLeft);
		noPixels = noPixels == 0 ? 1 : noPixels; // Ensures at least one point is sampled per row
		// Calculates points and corresponding texture points
		CanvasPoint leftPt = CanvasPoint(xLeft, row);
		CanvasPoint rightPt = CanvasPoint(xRight, row);
		setCanvasPointTexture(leftPt, left, bottom);
		setCanvasPointTexture(rightPt, right, bottom);
		// Calculates line equation defined by points
		float xDiff = rightPt.texturePoint.x - leftPt.texturePoint.x;
		float yDiff = rightPt.texturePoint.y - leftPt.texturePoint.y;
		float xStepSize = xDiff / noPixels;
		float yStepSize = yDiff / noPixels;
		// Samples each pixel from texture map and draws it
		for (float pix = 0.0; round(pix) <= noPixels; pix++) {
			int xText = round(leftPt.texturePoint.x + (xStepSize * pix));
			int yText = round(leftPt.texturePoint.y + (yStepSize * pix));
			window.setPixelColour(round(pix) + xLeft, row, texture.pixels[(yText * texture.width) + xText]);
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

void addColours(std::string mtlFileName, std::unordered_map<std::string, Colour>& cols) {
	// Loads mtl file
	std::ifstream mtlFile(mtlFileName);
	std::string line;
	// Col name and value
	std::string colName;
	uint32_t colVal;
	while (std::getline(mtlFile, line)) {
		std::vector<std::string> splitLine = split(line, ' ');
		// Reads colours into map
		if (splitLine.size() == 2 && splitLine[0] == "newmtl") {
			colName = splitLine[1];
		} else if (splitLine.size() == 4 && splitLine[0] == "Kd") {
			// Converts colours from [0.0, 1.0] -> [0, 255]
			int red = std::max(std::min(int(round(std::stof(splitLine[1]) * 255.0)), 255), 0);
			int green = std::max(std::min(int(round(std::stof(splitLine[2]) * 255.0)), 255), 0);
			int blue = std::max(std::min(int(round(std::stof(splitLine[3]) * 255.0)), 255), 0);
			// Adds colour to map
			cols[colName] = Colour(red, green, blue);
		}
	}
	// Closes file
	mtlFile.close();
}

std::unordered_map<std::string, Colour> loadColours(std::string objFileName) {
	std::unordered_map<std::string, Colour> cols;
	// Loads obj file
	std::ifstream objFile(objFileName);
	std::string line;
	while (std::getline(objFile, line)) {
		std::vector<std::string> splitLine = split(line, ' ');
		// Reads colours into map
		if (splitLine.size() == 2 && splitLine[0] == "mtllib") {
			addColours(splitLine[1], cols);
		}
	}
	// Closes file
	objFile.close();
	return cols;
}

std::vector<ModelTriangle> readObjFile(std::string filename, float scalingFactor) {
	std::vector<ModelTriangle> triangles;
	// Loads colour map
	std::unordered_map<std::string, Colour> colours = loadColours(filename);
	// Reads file line by line to build up list of vertices and triangles
	std::vector <glm::vec3> vertices;
	std::ifstream objFile(filename);
	std::string line;
	Colour col(255, 255, 255); // Stores current colour being used. Defaults to white
	while (std::getline(objFile, line)) {
		std::vector<std::string> splitLine = split(line, ' ');
		// Adds vertex to vertex list
		if (splitLine.size() == 4 && splitLine[0] == "v") {
			glm::vec3 vertex(std::stof(splitLine[1]), std::stof(splitLine[2]), std::stof(splitLine[3]));
			vertices.push_back(vertex * scalingFactor); // Adds scaled vertex
		} else if (splitLine.size() == 4 && splitLine[0] == "f") { // Adds face to triangles
			glm::vec3 vertex1 = vertices[std::stoi(split(splitLine[1], '/')[0]) - 1];
			glm::vec3 vertex2 = vertices[std::stoi(split(splitLine[2], '/')[0]) - 1];
			glm::vec3 vertex3 = vertices[std::stoi(split(splitLine[3], '/')[0]) - 1];
			triangles.push_back(ModelTriangle(vertex1, vertex2, vertex3, col));
		} else if (splitLine.size() == 2 && splitLine[0] == "usemtl") { // Sets colour
			col = colours[splitLine[1]];
		}

	}
	// Closes file
	objFile.close();
	return triangles;
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
		} else if (event.key.keysym.sym == SDLK_t) testTexturedTriangle(window);
	} else if (event.type == SDL_MOUSEBUTTONDOWN) {
		window.savePPM("output.ppm");
		window.saveBMP("output.bmp");
	}
}


int main(int argc, char *argv[]) {
	DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
	SDL_Event event;
	std::cout << readObjFile("cornell-box.obj", 1)[4] << std::endl;
	while (true) {
		// We MUST poll for events - otherwise the window will freeze !
		if (window.pollForInputEvents(event)) handleEvent(event, window);
		// Need to render the frame at the end, or nothing actually gets shown on the screen !
		window.renderFrame();
	}
}
