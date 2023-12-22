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

void drawLineDepth(DrawingWindow& window, CanvasPoint from, CanvasPoint to, Colour colour, float depthBuffer[HEIGHT][WIDTH]) {
	float xDiff = to.x - from.x;
	float yDiff = to.y - from.y;
	float numberOfSteps = abs(xDiff) > abs(yDiff) ? abs(xDiff) : abs(yDiff);
	// Handles case where drawLine is called from a point to that same point by drawing that point.
	if (numberOfSteps == 0) numberOfSteps = 1;
	float xStepSize = xDiff / numberOfSteps;
	float yStepSize = yDiff / numberOfSteps;
	float zStepSize = (to.depth - from.depth) / numberOfSteps;
	uint32_t col = (255 << 24) + (colour.red << 16) + (colour.green << 8) + colour.blue;
	for (float i = 0.0; i <= numberOfSteps; i++) {
		float x = from.x + (xStepSize * i);
		float y = from.y + (yStepSize * i);
		float z = from.depth + (zStepSize * i);
		// Draws pixel and updates z buffer if appropriate
		if (depthBuffer[(int)round(y)][(int)round(x)] < (1.0 / z)) {
			window.setPixelColour(round(x), round(y), col);
			depthBuffer[(int)round(y)][(int)round(x)] = 1.0 / z;
		}
	}
}

void drawStrokedTriangle(DrawingWindow& window, CanvasTriangle triangle, Colour colour) {
	drawLine(window, triangle.vertices[0], triangle.vertices[1], colour);
	drawLine(window, triangle.vertices[1], triangle.vertices[2], colour);
	drawLine(window, triangle.vertices[2], triangle.vertices[0], colour);
}

float interpolateDepth(CanvasPoint top, CanvasPoint bottom, float x, float y) {
	float distAlongLine = sqrt(((x - top.x) * (x - top.x)) + ((y - top.y) * (y - top.y)));
	float lineLength = sqrt(((bottom.x - top.x) * (bottom.x - top.x)) + ((bottom.y - top.y) * (bottom.y - top.y)));
	float z = top.depth + ((distAlongLine / lineLength) * (bottom.depth - top.depth));
	return z;
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
	CanvasPoint midArt = CanvasPoint(x, y, interpolateDepth(top, bottom, x, y));
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

void drawTopFlatTriangle(DrawingWindow& window, CanvasTriangle triangle, Colour colour, float depthBuffer[HEIGHT][WIDTH]) {
	CanvasPoint left = triangle.vertices[1];
	CanvasPoint right = triangle.vertices[2];
	CanvasPoint top = triangle.vertices[0];
	// Draws triangle
	float changeXPerRowLeft = (left.x - top.x) / (left.y - top.y);
	float changeXPerRowRight = (right.x - top.x) / (right.y - top.y);
	float xLeft = top.x;
	float xRight = top.x;
	for (int row = round(top.y); row < round(left.y); row++) {
		float dL = interpolateDepth(top, left, xLeft, row);
		float dR = interpolateDepth(top, right, xRight, row);
		drawLineDepth(window, CanvasPoint(round(xLeft), row, dL), CanvasPoint(round(xRight), row, dR), colour, depthBuffer);
		xLeft += changeXPerRowLeft;
		xRight += changeXPerRowRight;
	}
}

void drawBottomFlatTriangle(DrawingWindow& window, CanvasTriangle triangle, Colour colour, float depthBuffer[HEIGHT][WIDTH]) {
	CanvasPoint left = triangle.vertices[1];
	CanvasPoint right = triangle.vertices[2];
	CanvasPoint bottom = triangle.vertices[0];
	// Draws triangle
	float changeXPerRowLeft = (left.x - bottom.x) / (left.y - bottom.y);
	float changeXPerRowRight = (right.x - bottom.x) / (right.y - bottom.y);
	float xLeft = left.x;
	float xRight = right.x;
	for (int row = round(left.y); row < round(bottom.y); row++) {
		float dL = interpolateDepth(left, bottom, xLeft, row);
		float dR = interpolateDepth(right, bottom, xRight, row);
		drawLineDepth(window, CanvasPoint(round(xLeft), row, dL), CanvasPoint(round(xRight), row, dR), colour, depthBuffer);
		xLeft += changeXPerRowLeft;
		xRight += changeXPerRowRight;
	}
}

void drawFilledTriangle(DrawingWindow& window, CanvasTriangle triangle, Colour colour, float depthBuffer[HEIGHT][WIDTH]) {
	std::pair<CanvasTriangle, CanvasTriangle> tris = getTwoFlatTriangles(triangle);
	drawTopFlatTriangle(window, tris.first, colour, depthBuffer);
	drawBottomFlatTriangle(window, tris.second, colour, depthBuffer);
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

CanvasPoint getCanvasIntersectionPoint(glm::vec3 cameraPosition, glm::vec3 vertexPosition, float focalLength, float scale) {
	// Vertex position is in model coordinates (0, 0, 0) is centre of room.
	// Converts this to camera coordinate system (0, 0, 0) is the camera position. (note cameraPosition variable holds coordinates in model space too)
	glm::vec3 vertexPositionCameraSystem = vertexPosition - cameraPosition;
	// Calculates points on image plane (u, v)
	float u = -scale * focalLength * (vertexPositionCameraSystem.x / vertexPositionCameraSystem.z) + WIDTH / 2.0;
	float v = scale * focalLength * (vertexPositionCameraSystem.y / vertexPositionCameraSystem.z) + HEIGHT / 2.0;
	return CanvasPoint(u, v, -vertexPositionCameraSystem.z);
}


void drawPointCloud(DrawingWindow& window, std::string filename) {
	// Creates and zeros depth buffer
	float depthBuffer[HEIGHT][WIDTH];
	for (int y = 0; y < HEIGHT; y++) for (int x = 0; x < WIDTH; x++) depthBuffer[y][x] = 0.0;
	std::vector<ModelTriangle> model = readObjFile(filename, 0.35);
	glm::vec3 cameraPosition(0.0, 0.0, 4.0);
	float scaleFactor = 150.0;
	float focalLength = 2.0;
	for (ModelTriangle tri : model) {
		CanvasPoint vert1 = getCanvasIntersectionPoint(cameraPosition, tri.vertices[0], focalLength, scaleFactor);
		CanvasPoint vert2 = getCanvasIntersectionPoint(cameraPosition, tri.vertices[1], focalLength, scaleFactor);
		CanvasPoint vert3 = getCanvasIntersectionPoint(cameraPosition, tri.vertices[2], focalLength, scaleFactor);
		// Uncomment this line to draw stroked triangles
		//drawStrokedTriangle(window, CanvasTriangle(vert1, vert2, vert3), Colour(255, 255, 255));
		// Draws filled triangles
		drawFilledTriangle(window, CanvasTriangle(vert1, vert2, vert3), tri.colour, depthBuffer);
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


int main(int argc, char *argv[]) {
	DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
	SDL_Event event;
	while (true) {
		// We MUST poll for events - otherwise the window will freeze !
		if (window.pollForInputEvents(event)) handleEvent(event, window);
		drawPointCloud(window, "cornell-box.obj");
		// Need to render the frame at the end, or nothing actually gets shown on the screen !
		window.renderFrame();
	}
}
