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
#include <RayTriangleIntersection.h>

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
		if (round(y) >= 0.0 && round(y) < HEIGHT && round(x) >= 0.0 && round(x) < WIDTH && depthBuffer[(int)round(y)][(int)round(x)] < (1.0 / z)) {
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
	Colour col(255, 0, 0); // Stores current colour being used. Defaults to red
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
			ModelTriangle tri = ModelTriangle(vertex1, vertex2, vertex3, col);
			tri.normal = glm::normalize(glm::cross(vertex2 - vertex1, vertex3 - vertex1));
			triangles.push_back(tri);
		} else if (splitLine.size() == 2 && splitLine[0] == "usemtl") { // Sets colour
			col = colours[splitLine[1]];
		}
	}
	// Closes file
	objFile.close();
	return triangles;
}

CanvasPoint getCanvasIntersectionPoint(glm::vec3 cameraPosition, glm::vec3 vertexPosition, float focalLength, float scale, glm::mat3 cameraOrientation) {
	// Vertex position is in model coordinates (0, 0, 0) is centre of room.
	// Converts this to camera coordinate system (0, 0, 0) is the camera position. (note cameraPosition variable holds coordinates in model space too)
	glm::vec3 vertexPositionCameraSystem = cameraPosition - vertexPosition;
	// Adjusts intersection vector to account for camera orientation
	glm::vec3 vertexPosAdjustedOrientation = vertexPositionCameraSystem * cameraOrientation;
	// Calculates points on image plane (u, v)
	float u = -scale * focalLength * (vertexPosAdjustedOrientation.x / vertexPosAdjustedOrientation.z) + WIDTH / 2.0;
	float v = scale * focalLength * (vertexPosAdjustedOrientation.y / vertexPosAdjustedOrientation.z) + HEIGHT / 2.0;
	return CanvasPoint(u, v, vertexPosAdjustedOrientation.z);
}

glm::vec3 getCanvasPointDirectionFromCam(int x, int y, glm::vec3 cameraPosition, float focalLength, float scale, glm::mat3 cameraOrientation) {
	// Essentially does inverse of getCanvasIntersectionPoint
	float u = x;
	float v = y;
	// Calculates vertex points
	float vertZ = focalLength;
	float vertX = vertZ * (u - (WIDTH / 2.0)) / (scale * focalLength);
	float vertY = vertZ * (v - (HEIGHT / 2.0)) / (-scale * focalLength);
	glm::vec3 vertexPosAdjustedOrientation(vertX, vertY, vertZ);
	// Undoes orientation effect
	glm::vec3 vertexPositionCameraSystem = vertexPosAdjustedOrientation * glm::inverse(cameraOrientation);
	// Direction is point minus camera loc
	return glm::normalize(vertexPositionCameraSystem - cameraPosition);
}

glm::mat3 rotMatY(float theta) {
	glm::vec3 column0(std::cos(theta), 0.0, -std::sin(theta));
	glm::vec3 column1(0.0, 1.0, 0.0);
	glm::vec3 column2(std::sin(theta), 0.0, std::cos(theta));
	glm::mat3 rotMat(column0, column1, column2);
	return rotMat;
}

glm::vec3 rotateYAxis(glm::vec3 camPos, float theta) {
	return rotMatY(theta) * camPos;
}

void drawRasterisedScene(DrawingWindow& window, glm::vec3 cameraPosition, glm::mat3 camOrientation, std::vector<ModelTriangle> model,
	bool wireFrame, float focalLength, float scaleFactor) {
	window.clearPixels();
	// Creates and zeros depth buffer
	float depthBuffer[HEIGHT][WIDTH];
	for (int y = 0; y < HEIGHT; y++) for (int x = 0; x < WIDTH; x++) depthBuffer[y][x] = 0.0;
	for (ModelTriangle tri : model) {
		CanvasPoint vert1 = getCanvasIntersectionPoint(cameraPosition, tri.vertices[0], focalLength, scaleFactor, camOrientation);
		CanvasPoint vert2 = getCanvasIntersectionPoint(cameraPosition, tri.vertices[1], focalLength, scaleFactor, camOrientation);
		CanvasPoint vert3 = getCanvasIntersectionPoint(cameraPosition, tri.vertices[2], focalLength, scaleFactor, camOrientation);
		if (wireFrame) drawStrokedTriangle(window, CanvasTriangle(vert1, vert2, vert3), Colour(255, 255, 255));
		else drawFilledTriangle(window, CanvasTriangle(vert1, vert2, vert3), tri.colour, depthBuffer);
	}
}

std::pair<RayTriangleIntersection, bool> getClosestValidIntersection(glm::vec3 cameraPosition, glm::vec3 rayDirection, std::vector<ModelTriangle> model, 
	int triIndSpec /* Varaible used for shadow acne removal. -1 if not needed*/, float minT /* Used for shadow acne removal. triIndSpec = -1 if not needed*/) {
	RayTriangleIntersection closestIntersection;
	bool solFound = false;
	for (int i = 0; i < model.size(); i++) {
		// Handles case of shadow acne by not returning triangle which is being looked at
		if (triIndSpec != -1 && i == triIndSpec) continue;
		ModelTriangle triangle = model[i];
		// Calculates intersection between camera and ray
		glm::vec3 e0 = triangle.vertices[1] - triangle.vertices[0];
		glm::vec3 e1 = triangle.vertices[2] - triangle.vertices[0];
		glm::vec3 SPVector = cameraPosition - triangle.vertices[0];
		glm::mat3 DEMatrix(-rayDirection, e0, e1);
		glm::vec3 possibleSolution = glm::inverse(DEMatrix) * SPVector; // of form [t, u, v]
		float t = possibleSolution[0];
		float u = possibleSolution[1];
		float v = possibleSolution[2];
		// Checks to see if intersection is valid
		bool isValid = (u >= 0.0) && (u <= 1.0) && (v >= 0.0) && (v <= 1.0) && (u + v) <= 1.0 && t >= 0.0;
		if (isValid && (!solFound || closestIntersection.distanceFromCamera > t) && (t >= minT || triIndSpec == -1)) {
			solFound = true;
			// Calculates r. (Alternatively could use something like glm::vec3 r = cameraPosition + t * rayDirection;)
			glm::vec3 r = triangle.vertices[0] + u * e0 + v * e1;
			closestIntersection = RayTriangleIntersection(r, t, triangle, i);
		}
	}
	return std::pair<RayTriangleIntersection, bool>(closestIntersection, solFound);
}

float proximityLighting(RayTriangleIntersection intersection, glm::vec3 lightPos) {
	glm::vec3 intToLight = glm::normalize(lightPos - intersection.intersectionPoint);
	// Implements proximity lighting
	float distFromLight = glm::distance(lightPos, intersection.intersectionPoint);
	float proxIntensity = 1.0;
	if (distFromLight > 0.0) proxIntensity = (1.0 * 20.0) / (4.0 * M_PI * distFromLight * distFromLight);
	if (proxIntensity > 1.0) proxIntensity = 1.0;
	if (proxIntensity < 0.0) proxIntensity = 0.0;
	return proxIntensity;
}

float angleOfIncidenceLighting(glm::vec3 point, glm::vec3 normal, glm::vec3 lightPos) {
	glm::vec3 intToLight = glm::normalize(lightPos - point);
	// Angle of incidence lighting
	float angleIntensity = glm::dot(normal, intToLight);
	if (angleIntensity > 1.0) angleIntensity = 1.0;
	if (angleIntensity < 0.0) angleIntensity = 0.0;
	return angleIntensity;
}

std::array<float, 3> baryCoords(ModelTriangle tri, glm::vec3 p) {
	// Sets up barycentric vars to make code more readable
	glm::vec3 a = tri.vertices[0];
	glm::vec3 b = tri.vertices[1];
	glm::vec3 c = tri.vertices[2];
	// Calculates subtriangle areas
	float areaABC = glm::dot(tri.normal, glm::cross(b - a, c - a));
	float areaBCP = glm::dot(tri.normal, glm::cross(b - p, c - p));
	float areaCAP = glm::dot(tri.normal, glm::cross(c - p, a - p));
	// Calculates barycentric coords
	float ba = areaBCP / areaABC;
	float bb = areaCAP / areaABC;
	float bc = 1.0f - ba - bb;
	// Creates array to return
	std::array<float, 3> baryCoords;
	baryCoords[0] = ba;
	baryCoords[1] = bb;
	baryCoords[2] = bc;
	return baryCoords;
}

glm::vec3 interpolateVertNormalPhong(RayTriangleIntersection intersection, std::array<glm::vec3, 3> vertNorm) {
	std::array<float, 3> bCoords = baryCoords(intersection.intersectedTriangle, intersection.intersectionPoint);
	glm::vec3 n = glm::normalize(bCoords[0] * vertNorm[0] + bCoords[1] * vertNorm[1] + bCoords[2] * vertNorm[2]);
	return n;
}

float angleOfIncidenceLightingGourad(RayTriangleIntersection intersection, glm::vec3 lightPos, std::vector<std::array<glm::vec3, 3>> vertNorms) {
	// Caclulate brightness for each vertex
	float ia = angleOfIncidenceLighting(intersection.intersectedTriangle.vertices[0], vertNorms[intersection.triangleIndex][0], lightPos);
	float ib = angleOfIncidenceLighting(intersection.intersectedTriangle.vertices[1], vertNorms[intersection.triangleIndex][1], lightPos);
	float ic = angleOfIncidenceLighting(intersection.intersectedTriangle.vertices[2], vertNorms[intersection.triangleIndex][2], lightPos);
	std::array<float, 3> bCoords = baryCoords(intersection.intersectedTriangle, intersection.intersectionPoint);
	// Caclulates final intensity
	float intensity = ia * bCoords[0] + ib * bCoords[1] + ic * bCoords[2];
	return intensity;
}

float angleOfIncidenceLightingPhong(RayTriangleIntersection intersection, glm::vec3 lightPos, std::vector<std::array<glm::vec3, 3>> vertNorms) {
	glm::vec3 n = interpolateVertNormalPhong(intersection, vertNorms[intersection.triangleIndex]);
	float intensity = angleOfIncidenceLighting(intersection.intersectionPoint, n, lightPos);
	return intensity;
}


float specularLighting(glm::vec3 p, glm::vec3 normal, glm::vec3 lightPos, glm::vec3 cameraPosition) {
	int exponent = 256; // Exponent used for specular lighting calc. Smaller value leads to more spread out light.
	glm::vec3 incidentVector = glm::normalize(p - lightPos); // Vector from light source to point
	glm::vec3 pointToEyeVector = glm::normalize(cameraPosition - p); // Vector from point to camera
	glm::vec3 reflectionVec = glm::normalize(incidentVector -  normal * 2.0f * (glm::dot(incidentVector, normal)));
	float intensity = pow(glm::dot(pointToEyeVector, reflectionVec), exponent);
	if (intensity > 1.0) intensity = 1.0;
	if (intensity < 0.0) intensity = 0.0;
	return intensity;
}

float specularLightingGourad(RayTriangleIntersection intersection, glm::vec3 lightPos, std::vector<std::array<glm::vec3, 3>> vertNorms, glm::vec3 cameraPosition) {
	// Caclulate brightness for each vertex
	float ia = specularLighting(intersection.intersectedTriangle.vertices[0], vertNorms[intersection.triangleIndex][0], lightPos, cameraPosition);
	float ib = specularLighting(intersection.intersectedTriangle.vertices[1], vertNorms[intersection.triangleIndex][1], lightPos, cameraPosition);
	float ic = specularLighting(intersection.intersectedTriangle.vertices[2], vertNorms[intersection.triangleIndex][2], lightPos, cameraPosition);
	std::array<float, 3> bCoords = baryCoords(intersection.intersectedTriangle, intersection.intersectionPoint);
	// Caclulates final intensity
	float intensity = ia * bCoords[0] + ib * bCoords[1] + ic * bCoords[2];
	return intensity;
}

float specularLightingPhong(RayTriangleIntersection intersection, glm::vec3 lightPos, std::vector<std::array<glm::vec3, 3>> vertNorms, glm::vec3 cameraPosition) {
	glm::vec3 n = interpolateVertNormalPhong(intersection, vertNorms[intersection.triangleIndex]);
	float intensity = specularLighting(intersection.intersectionPoint, n, lightPos, cameraPosition);
	return intensity;
}

float shadowCodeWeek6(RayTriangleIntersection intersection, glm::vec3 lightPos, std::vector<ModelTriangle> model) {
	float intensity = 0.0;
	// Checks to see if pixel should be in shadow
	std::pair<RayTriangleIntersection, bool> shadowCheck = getClosestValidIntersection(intersection.intersectionPoint,
		glm::normalize(lightPos - intersection.intersectionPoint), model, intersection.triangleIndex, 0.1);
	if (!shadowCheck.second || glm::distance(intersection.intersectionPoint, shadowCheck.first.intersectionPoint) >=
		glm::distance(intersection.intersectionPoint, lightPos)) intensity = 1.0;
	return intensity;
}

Colour scaleIntensity(float intensity, Colour col, float minIntensity) {
	if (intensity > 1.0f) intensity = 1.0f;
	if (intensity < minIntensity) intensity = minIntensity; // Ambient lighting
	Colour colour = Colour(round(col.red * intensity), round(col.green * intensity), round(col.blue * intensity));
	return colour;
}

std::vector<std::array<glm::vec3, 3>> caclulateVertexNormals(std::vector<ModelTriangle> model) {
	// Calculates vertex normal for each vertex. Could easily make this algorithm faster but no need
	// as I'm not doing CW for this unit.
	std::vector<std::array<glm::vec3, 3>> vertexNorms;
	for (int triIndex = 0; triIndex < model.size(); triIndex++) {
		ModelTriangle tri = model[triIndex];
		std::array<glm::vec3, 3> normsForTri;
		for (int vert = 0; vert < 3; vert++) {
			glm::vec3 vertex = tri.vertices[vert];
			int noVertexMatches = 1;
			normsForTri[vert] = tri.normal;
			// Searches all other triangles for matching vertices and updates normals as required (ineffecient but easy code).
			for (int triI = 0; triI < model.size(); triI++) {
				if (triI == triIndex) continue;
				for (int vertI = 0; vertI < 3; vertI++) {
					// Note == normally should be epsilon equals for float comparison but equal vertices will be exactly the same in this case..
					// May be worth replacing with epsion equals in future potentially.
					if (vertex == model[triI].vertices[vertI]) {
						normsForTri[vert] += model[triI].vertices[vertI];
						noVertexMatches++;
					}
				}
			}
			normsForTri[vert] /= (float)noVertexMatches;
			normsForTri[vert] == glm::normalize(normsForTri[vert]);
		}
		vertexNorms.push_back(normsForTri);
	}
	return vertexNorms;
}

void drawRaytracedScene(DrawingWindow& window, glm::vec3 cameraPosition, glm::mat3 camOrientation, std::vector<ModelTriangle> model,
	float focalLength, float scaleFactor, std::vector<std::array<glm::vec3, 3>> vertNorms) {
	window.clearPixels();
	glm::vec3 lightPos = (glm::vec3(0.0, 2.7, 0.0) * 0.35f);
	//glm::vec3 lightPos = (glm::vec3(0.0, 2.7, 3.0) * 0.35f);
	//model.push_back(ModelTriangle(lightPos + glm::vec3(0.0, -0.25, 0.0), lightPos + glm::vec3(-0.25, 0.25, 0.0), lightPos + glm::vec3(0.25, 0.25, 0.0), Colour(0, 255, 0)));
	// Loops through each pixel of image plane. Traces ray and draws colour if apprporiate
	for (int y = 0; y < HEIGHT; y++) {
		for (int x = 0; x < WIDTH; x++) {
			glm::vec3 dir = getCanvasPointDirectionFromCam(x, y, cameraPosition, focalLength, scaleFactor, camOrientation);
			std::pair<RayTriangleIntersection, bool> intersection = getClosestValidIntersection(cameraPosition, dir, model, -1, 0.0);
			if (intersection.second) {
				float diffuseLighting = 0.1f * proximityLighting(intersection.first, lightPos) +
					0.9f * angleOfIncidenceLightingPhong(intersection.first, lightPos, vertNorms);
				float specularLighting = specularLightingPhong(intersection.first, lightPos, vertNorms, cameraPosition);
				float intensity = diffuseLighting + specularLighting;
				Colour colour = scaleIntensity(intensity, intersection.first.intersectedTriangle.colour, 0.2);
				// Draws raytraced pixel
				uint32_t col = (255 << 24) + (colour.red << 16) + (colour.green << 8) + colour.blue;
				window.setPixelColour(x, y, col);
			}
		}
	}
}

void draw(DrawingWindow& window, std::vector<ModelTriangle> model, glm::vec3* camPos, glm::mat3* cameraOrientation, bool orbit, bool wireFrame, 
	bool rayTracing, std::vector<std::array<glm::vec3, 3>> vertNorms) {
	float scaleFactor = 150.0;
	float focalLength = 2.0;
	if (rayTracing) drawRaytracedScene(window, *camPos, *cameraOrientation, model, focalLength, scaleFactor, vertNorms);
	else drawRasterisedScene(window, *camPos, *cameraOrientation, model, wireFrame, focalLength, scaleFactor);
	if (orbit) {
		// Rotates camera after every draw a little bit
		*camPos = rotateYAxis(*camPos, -0.01);
		// Updates orientation accoringly to achieve orbit effect
		glm::vec3 lookAtPoint(0.0, 0.0, 0.0); // Looks at centre
		glm::vec3 forward = *camPos - lookAtPoint;
		glm::vec3 right = glm::cross(glm::vec3(0.0, 1.0, 0.0), forward);
		glm::vec3 up = glm::cross(forward, right);
		*cameraOrientation = glm::mat3(glm::normalize(right), glm::normalize(up), glm::normalize(forward));
	}
}

glm::vec3 translate(glm::vec3 camPos, glm::vec3 trans) {
	return camPos + trans;
}

glm::mat3 rotMatX(float theta) {
	glm::vec3 column0(1.0, 0.0, 0.0);
	glm::vec3 column1(0.0, std::cos(theta), std::sin(theta));
	glm::vec3 column2(0.0, -std::sin(theta), std::cos(theta));
	glm::mat3 rotMat(column0, column1, column2);
	return rotMat;
}

glm::vec3 rotateXAxis(glm::vec3 camPos, float theta) {
	return rotMatX(theta) * camPos;
}

glm::mat3 rotateYAxis(glm::mat3 camOrientation, float theta) {
	return rotMatY(theta) * camOrientation;
}


glm::mat3 rotateXAxis(glm::mat3 camOrientation, float theta) {
	return rotMatX(theta) * camOrientation;
}


void handleEvent(SDL_Event event, DrawingWindow &window, glm::vec3 *cameraPosition, glm::mat3* cameraOrientation, bool* orbit, bool* wireFrame,
	bool* rayTracing) {
	if (event.type == SDL_KEYDOWN) {
		if (event.key.keysym.sym == SDLK_LEFT) {
			std::cout << "LEFT" << std::endl;
			*cameraPosition = translate(*cameraPosition, glm::vec3(0.1, 0, 0));
		}
		else if (event.key.keysym.sym == SDLK_RIGHT) {
			std::cout << "RIGHT" << std::endl;
			*cameraPosition = translate(*cameraPosition, glm::vec3(-0.1, 0, 0));
		}
		else if (event.key.keysym.sym == SDLK_UP) {
			std::cout << "UP" << std::endl;
			*cameraPosition = translate(*cameraPosition, glm::vec3(0, -0.1, 0));
		}
		else if (event.key.keysym.sym == SDLK_DOWN) {
			std::cout << "DOWN" << std::endl;
			*cameraPosition = translate(*cameraPosition, glm::vec3(0, 0.1, 0));
		}
		else if (event.key.keysym.sym == SDLK_f) {
			std::cout << "FORWARD" << std::endl;
			*cameraPosition = translate(*cameraPosition, glm::vec3(0, 0, -0.1));
		}
		else if (event.key.keysym.sym == SDLK_b) {
			std::cout << "BACKWARD" << std::endl;
			*cameraPosition = translate(*cameraPosition, glm::vec3(0, 0, 0.1));
		}
		else if (event.key.keysym.sym == SDLK_x) {
			std::cout << "X Rotation" << std::endl;
			*cameraPosition = rotateXAxis(*cameraPosition, 0.05);
		}
		else if (event.key.keysym.sym == SDLK_y) {
			std::cout << "Y Rotation" << std::endl;
			*cameraPosition = rotateYAxis(*cameraPosition, 0.05);
		}
		else if (event.key.keysym.sym == SDLK_p) {
			std::cout << "PAN" << std::endl;
			*cameraOrientation = rotateYAxis(*cameraOrientation, 0.05);
		}
		else if (event.key.keysym.sym == SDLK_t) {
			std::cout << "TILT" << std::endl;
			*cameraOrientation = rotateXAxis(*cameraOrientation, 0.05);
		}
		else if (event.key.keysym.sym == SDLK_o) {
			std::cout << "Orbit Mode Changed" << std::endl;
			*orbit = !*orbit;
		}
		else if (event.key.keysym.sym == SDLK_w) {
			std::cout << "Wireframe Mode Changed" << std::endl;
			*wireFrame = !*wireFrame;
		} else if (event.key.keysym.sym == SDLK_r) {
			*rayTracing = !*rayTracing;
			std::cout << "Rendering mode changed. Raytracing (1 means raytracin): " << *rayTracing << std::endl;
		}
	} else if (event.type == SDL_MOUSEBUTTONDOWN) {
		window.savePPM("output.ppm");
		window.saveBMP("output.bmp");
	}
}


int main(int argc, char *argv[]) {
	DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
	SDL_Event event;
	glm::vec3 cameraPosition(0.0, 0.0, 4.0);
	glm::mat3 cameraOrientation(1.0);
	std::vector<ModelTriangle> model = readObjFile("cornell-box.obj", 0.35);
	std::vector<std::array<glm::vec3, 3>> vertNorms = caclulateVertexNormals(model);
	bool orbit = false;
	bool wireFrame = false;
	bool rayTracing = true;
	while (true) {
		// We MUST poll for events - otherwise the window will freeze !
		if (window.pollForInputEvents(event)) handleEvent(event, window, &cameraPosition, &cameraOrientation, &orbit, &wireFrame, &rayTracing);
		draw(window, model, &cameraPosition, &cameraOrientation, orbit, wireFrame, rayTracing, vertNorms);
		// Need to render the frame at the end, or nothing actually gets shown on the screen !
		window.renderFrame();
	}
}
