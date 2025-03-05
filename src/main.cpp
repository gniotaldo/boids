#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <glm/gtc/type_ptr.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <cfloat>  
#include <chrono>
#include <thread>

#include "Shader_Loader.h" 

GLuint program, programDepth;
Core::Shader_Loader shaderLoader;

float moveSpeed = 10.f;

glm::vec3 cameraPos = glm::vec3(0.f, 1.f, 40.f);
glm::vec3 cameraDir = glm::vec3(0.f, 0.f, -1.f);
glm::vec3 up = glm::vec3(0.f, 1.f, 0.f);

float lastX = 500.0f, lastY = 500.0f;
bool firstMouse = true;
float yaw = -90.0f;   
float pitch = 0.0f;
float mouseSensitivity = 0.1f;


bool shadowMappingEnabled = true;
bool normalMappingEnabled = true;

bool zKeyPressed = false;
bool xKeyPressed = false;


GLuint fishVAO, fishVBO;
GLuint borderVAO, borderVBO;
GLuint floorVAO, floorVBO, floorEBO;
int floorIndexCount = 0;
GLuint cactusVAO, cactusVBO;
GLuint algaeVAO, algaeVBO;
GLuint submarineVAO, submarineVBO;  

GLuint depthMapFBO, depthMap;
float aspectRatio = 1.f;
glm::mat4 lightSpaceMatrix;
const GLuint SHADOW_WIDTH = 1024, SHADOW_HEIGHT = 1024;

float lastTime = -1.f;
float deltaTime = 0.f;


struct Light {
    glm::vec3 position;
    glm::vec3 color;
    float intensity;
};

Light topLight = {
    glm::vec3(10.0f, 15.0f, 10.0f),
    glm::vec3(1.0f, 1.0f, 1.0f),
    10.0f
};

// wagi
float separationWeight = 2.0f;
float alignmentWeight = 4.f;
float cohesionWeight = 2.0f;
float boundaryAvoidanceWeight = 10.0f;


GLuint floorDiffuseTexture, floorNormalTexture;


float gTerrainSize;
int   gTerrainResolution;
float gTerrainAmplitude;
float gTerrainBase;
float gTerrainOffsetX, gTerrainOffsetZ;

enum ObstacleType { CACTUS, ALGAE };

struct Obstacle {
    ObstacleType type;
    glm::vec3 position;
    glm::vec3 scale;
    float rotation;
};

std::vector<Obstacle> obstacles; 


struct Player {
    glm::vec3 position;
    float attractionWeight;  // przyciaganie rybek do gracza
};

Player player;

// ----------- Kolizje -------------
struct AABB {
    glm::vec3 min;
    glm::vec3 max;
};

AABB transformAABB(const glm::mat4& model, const glm::vec3& localMin, const glm::vec3& localMax) {
    AABB aabb;
    glm::vec3 corners[8] = {
        glm::vec3(localMin.x, localMin.y, localMin.z),
        glm::vec3(localMin.x, localMin.y, localMax.z),
        glm::vec3(localMin.x, localMax.y, localMin.z),
        glm::vec3(localMin.x, localMax.y, localMax.z),
        glm::vec3(localMax.x, localMin.y, localMin.z),
        glm::vec3(localMax.x, localMin.y, localMax.z),
        glm::vec3(localMax.x, localMax.y, localMin.z),
        glm::vec3(localMax.x, localMax.y, localMax.z)
    };

    glm::vec3 minPt(FLT_MAX);
    glm::vec3 maxPt(-FLT_MAX);
    for (int i = 0; i < 8; i++) {
        glm::vec4 transformed = model * glm::vec4(corners[i], 1.0f);
        glm::vec3 pos = glm::vec3(transformed);
        minPt = glm::min(minPt, pos);
        maxPt = glm::max(maxPt, pos);
    }
    aabb.min = minPt;
    aabb.max = maxPt;
    return aabb;
}

bool aabbIntersect(const AABB& a, const AABB& b) {
    return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
        (a.min.y <= b.max.y && a.max.y >= b.min.y) &&
        (a.min.z <= b.max.z && a.max.z >= b.min.z);
}

const glm::vec3 fishLocalMin(-0.25f, 0.0f, -0.25f);
const glm::vec3 fishLocalMax(0.25f, 0.5f, 0.25f);
const glm::vec3 cactusLocalMin(-0.5f, -3.0f, -0.5f);
const glm::vec3 cactusLocalMax(0.5f, 3.0f, 0.5f);
const glm::vec3 algaeLocalMin(-0.25f, -6.0f, -0.05f);
const glm::vec3 algaeLocalMax(0.25f, 0.0f, 0.05f);

float terrainNoise(float x, float z) {
    return sin(x * 0.5f + 1.0f) * cos(z * 0.5f + 1.0f);
}

// ----------- Tekstury -------------
GLuint loadTexture(const char* path)
{
    GLuint textureID;
    glGenTextures(1, &textureID);
    int width, height, nrComponents;
    stbi_set_flip_vertically_on_load(true); 
    unsigned char* data = stbi_load(path, &width, &height, &nrComponents, 0);
    if (data)
    {
        GLenum format;
        if (nrComponents == 1)
            format = GL_RED;
        else if (nrComponents == 3)
            format = GL_RGB;
        else if (nrComponents == 4)
            format = GL_RGBA;
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        stbi_image_free(data);
    }
    else
    {
        std::cout << "Texture failed to load at path: " << path << std::endl;
        stbi_image_free(data);
    }
    return textureID;
}

// ----------- Shadow mapping -------------
void initShadowMap() {
    glGenFramebuffers(1, &depthMapFBO);
    glGenTextures(1, &depthMap);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, SHADOW_WIDTH, SHADOW_HEIGHT,
        0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    float borderColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void calculateLightSpaceMatrix() {
    float near_plane = 1.0f, far_plane = 100.0f;
    float orthoSize = 30.0f;
    glm::mat4 lightProjection = glm::ortho(-orthoSize, orthoSize, -orthoSize, orthoSize, near_plane, far_plane);
    glm::mat4 lightView = glm::lookAt(topLight.position,
        glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, 1.0f, 0.0f));
    lightSpaceMatrix = lightProjection * lightView;
}

// ----------- Œwiat³o -------------
void setLightUniforms(GLuint shaderProgram, const glm::vec3& cameraPos) {
    glUseProgram(shaderProgram);
    GLint lightPosLoc = glGetUniformLocation(shaderProgram, "light.position");
    GLint lightColorLoc = glGetUniformLocation(shaderProgram, "light.color");
    GLint lightIntensityLoc = glGetUniformLocation(shaderProgram, "light.intensity");
    GLint viewPosLoc = glGetUniformLocation(shaderProgram, "viewPos");
    if (lightPosLoc != -1)
        glUniform3fv(lightPosLoc, 1, glm::value_ptr(topLight.position));
    if (lightColorLoc != -1)
        glUniform3fv(lightColorLoc, 1, glm::value_ptr(topLight.color));
    if (lightIntensityLoc != -1)
        glUniform1f(lightIntensityLoc, topLight.intensity);
    if (viewPosLoc != -1)
        glUniform3fv(viewPosLoc, 1, glm::value_ptr(cameraPos));
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos) {
    if (firstMouse) {
        lastX = float(xpos);
        lastY = float(ypos);
        firstMouse = false;
    }
    float xoffset = float(xpos) - lastX;
    float yoffset = lastY - float(ypos);
    lastX = float(xpos);
    lastY = float(ypos);

    xoffset *= mouseSensitivity;
    yoffset *= mouseSensitivity;

    yaw += xoffset;
    pitch += yoffset;

    if (pitch > 89.0f)
        pitch = 89.0f;
    if (pitch < -89.0f)
        pitch = -89.0f;

    glm::vec3 direction;
    direction.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    direction.y = sin(glm::radians(pitch));
    direction.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    cameraDir = glm::normalize(direction);
}

// ----------- Klasa ryb -------------
class Fish {
public:
    glm::vec3 position;
    glm::vec3 velocity;
    glm::quat orientation;
    float maxSpeed = 5.0f;
    float maxForce = 0.1f;
    float perceptionRadius = 3.0f;
    glm::vec3 color;

    Fish(glm::vec3 pos)
        : position(pos),
        color(glm::vec3(std::abs(std::sin(pos.x)),
            std::abs(std::cos(pos.y)),
            std::abs(std::sin(pos.z)))),
        velocity((rand() % 100 - 50) * 0.1f,
            (rand() % 100 - 50) * 0.1f,
            (rand() % 100 - 50) * 0.1f)
    {
        orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    }


    glm::vec3 boundaryAvoidance() {
        float boundaryLimit = 9.5f;
        glm::vec3 avoidanceForce(0.0f);
        if (position.x > boundaryLimit)
            avoidanceForce.x = -(position.x - boundaryLimit);
        if (position.x < -boundaryLimit)
            avoidanceForce.x = -position.x - boundaryLimit;
        if (position.y > boundaryLimit)
            avoidanceForce.y = -(position.y - boundaryLimit);
        if (position.y < -boundaryLimit)
            avoidanceForce.y = -position.y - boundaryLimit;
        if (position.z > boundaryLimit)
            avoidanceForce.z = -(position.z - boundaryLimit);
        if (position.z < -boundaryLimit)
            avoidanceForce.z = -position.z - boundaryLimit;
        if (glm::length(avoidanceForce) > 0) {
            avoidanceForce = glm::normalize(avoidanceForce) * maxSpeed;
            avoidanceForce -= velocity;
            avoidanceForce = glm::clamp(avoidanceForce, -maxForce, maxForce);
        }
        return avoidanceForce;
    }

    glm::vec3 separation(const std::vector<Fish>& fishes) {
        glm::vec3 separationForce(0.0f);
        int nearbyCount = 0;
        for (const auto& other : fishes) {
            float distance = glm::length(position - other.position);
            if (distance > 0 && distance < perceptionRadius) {
                glm::vec3 diff = position - other.position;
                separationForce += glm::normalize(diff) / distance;
                nearbyCount++;
            }
        }
        if (nearbyCount > 0) {
            separationForce /= nearbyCount;
            separationForce = glm::normalize(separationForce) * maxSpeed - velocity;
            separationForce = glm::clamp(separationForce, -maxForce, maxForce);
        }
        return separationForce;
    }

    glm::vec3 alignment(const std::vector<Fish>& fishes) {
        glm::vec3 alignmentForce(0.0f);
        int nearbyCount = 0;
        for (const auto& other : fishes) {
            float distance = glm::length(position - other.position);
            if (distance > 0 && distance < perceptionRadius) {
                alignmentForce += other.velocity;
                nearbyCount++;
            }
        }
        if (nearbyCount > 0) {
            alignmentForce /= nearbyCount;
            alignmentForce = glm::normalize(alignmentForce) * maxSpeed - velocity;
            alignmentForce = glm::clamp(alignmentForce, -maxForce, maxForce);
        }
        return alignmentForce;
    }

    glm::vec3 cohesion(const std::vector<Fish>& fishes) {
        glm::vec3 cohesionForce(0.0f);
        glm::vec3 centerOfMass(0.0f);
        int nearbyCount = 0;
        for (const auto& other : fishes) {
            float distance = glm::length(position - other.position);
            if (distance > 0 && distance < perceptionRadius) {
                centerOfMass += other.position;
                nearbyCount++;
            }
        }
        if (nearbyCount > 0) {
            centerOfMass /= nearbyCount;
            cohesionForce = centerOfMass - position;
            cohesionForce = glm::normalize(cohesionForce) * maxSpeed - velocity;
            cohesionForce = glm::clamp(cohesionForce, -maxForce, maxForce);
        }
        return cohesionForce;
    }

    void update(const std::vector<Fish>& fishes, float deltaTime, const glm::vec3& playerPos, float playerAttractionWeight) {
        glm::vec3 sep = separation(fishes) * separationWeight;
        glm::vec3 ali = alignment(fishes) * alignmentWeight;
        glm::vec3 coh = cohesion(fishes) * cohesionWeight;
        glm::vec3 bound = boundaryAvoidance() * boundaryAvoidanceWeight;
        glm::vec3 toPlayer = playerPos - position;
        glm::vec3 attraction(0.0f);
        if (glm::length(toPlayer) > 0.001f) {
            attraction = glm::normalize(toPlayer) * maxSpeed - velocity;
            attraction = glm::clamp(attraction, -maxForce, maxForce);
        }
        velocity += sep + ali + coh + bound + (playerAttractionWeight * attraction);
        if (glm::length(velocity) > maxSpeed)
            velocity = glm::normalize(velocity) * maxSpeed;
        position += velocity * deltaTime;
        glm::vec3 desiredDir = glm::normalize(velocity);
        if (glm::length(desiredDir) > 0.001f) {
            glm::quat desiredOrientation = glm::rotation(glm::vec3(0, 0, -1), desiredDir);
            float turningSpeed = 2.0f;
            orientation = glm::slerp(orientation, desiredOrientation, turningSpeed * deltaTime);
        }
    }
};

std::vector<Fish> fishes;
const int NUM_FISHES = 50;

// ----------- Tworzenie geometrii -------------
std::vector<float> get_pyramid_vertices(float width, float height, float depth) {
    float w2 = width / 2.0f;
    float d2 = depth / 2.0f;
    std::vector<float> vertices;
    // podstawa
    vertices.insert(vertices.end(), { -w2, 0.0f, -d2,   0.0f, -1.0f, 0.0f });
    vertices.insert(vertices.end(), { w2, 0.0f, -d2,    0.0f, -1.0f, 0.0f });
    vertices.insert(vertices.end(), { w2, 0.0f,  d2,    0.0f, -1.0f, 0.0f });
    vertices.insert(vertices.end(), { w2, 0.0f,  d2,    0.0f, -1.0f, 0.0f });
    vertices.insert(vertices.end(), { -w2, 0.0f,  d2,   0.0f, -1.0f, 0.0f });
    vertices.insert(vertices.end(), { -w2, 0.0f, -d2,   0.0f, -1.0f, 0.0f });
    // sciany boczne
    glm::vec3 n1 = glm::normalize(glm::vec3(0.0f, -d2, height));
    glm::vec3 n2 = glm::normalize(glm::vec3(w2, -height, 0.0f));
    glm::vec3 n3 = glm::normalize(glm::vec3(0.0f, -d2, -height));
    glm::vec3 n4 = glm::normalize(glm::vec3(-w2, -height, 0.0f));
    
    vertices.insert(vertices.end(), { 0.0f, height, 0.0f, n1.x, n1.y, n1.z });
    vertices.insert(vertices.end(), { -w2, 0.0f, -d2,  n1.x, n1.y, n1.z });
    vertices.insert(vertices.end(), { w2, 0.0f, -d2,   n1.x, n1.y, n1.z });
    
    vertices.insert(vertices.end(), { 0.0f, height, 0.0f, n2.x, n2.y, n2.z });
    vertices.insert(vertices.end(), { w2, 0.0f, -d2,   n2.x, n2.y, n2.z });
    vertices.insert(vertices.end(), { w2, 0.0f,  d2,   n2.x, n2.y, n2.z });
    
    vertices.insert(vertices.end(), { 0.0f, height, 0.0f, n3.x, n3.y, n3.z });
    vertices.insert(vertices.end(), { w2, 0.0f,  d2,   n3.x, n3.y, n3.z });
    vertices.insert(vertices.end(), { -w2, 0.0f,  d2,   n3.x, n3.y, n3.z });
    
    vertices.insert(vertices.end(), { 0.0f, height, 0.0f, n4.x, n4.y, n4.z });
    vertices.insert(vertices.end(), { -w2, 0.0f,  d2,   n4.x, n4.y, n4.z });
    vertices.insert(vertices.end(), { -w2, 0.0f, -d2,  n4.x, n4.y, n4.z });
    return vertices;
}

std::vector<float> get_cube_vertices(float width, float height, float depth) {
    float w = width / 2, h = height / 2, d = depth / 2;
    std::vector<float> vertices = {
        
        -w, -h,  d,    0, 0, 1,
         w, -h,  d,    0, 0, 1,
         w,  h,  d,    0, 0, 1,
         w,  h,  d,    0, 0, 1,
        -w,  h,  d,    0, 0, 1,
        -w, -h,  d,    0, 0, 1,
        
        -w, -h, -d,    0, 0, -1,
        -w,  h, -d,    0, 0, -1,
         w,  h, -d,    0, 0, -1,
         w,  h, -d,    0, 0, -1,
         w, -h, -d,    0, 0, -1,
        -w, -h, -d,    0, 0, -1,
        
        -w,  h,  d,   -1, 0, 0,
        -w,  h, -d,   -1, 0, 0,
        -w, -h, -d,   -1, 0, 0,
        -w, -h, -d,   -1, 0, 0,
        -w, -h,  d,   -1, 0, 0,
        -w,  h,  d,   -1, 0, 0,
        
         w,  h,  d,    1, 0, 0,
         w, -h, -d,    1, 0, 0,
         w,  h, -d,    1, 0, 0,
         w, -h, -d,    1, 0, 0,
         w,  h,  d,    1, 0, 0,
         w, -h,  d,    1, 0, 0,
         
         -w,  h, -d,    0, 1, 0,
         -w,  h,  d,    0, 1, 0,
          w,  h,  d,    0, 1, 0,
          w,  h,  d,    0, 1, 0,
          w,  h, -d,    0, 1, 0,
         -w,  h, -d,    0, 1, 0,
         
         -w, -h, -d,    0, -1, 0,
          w, -h, -d,    0, -1, 0,
          w, -h,  d,    0, -1, 0,
          w, -h,  d,    0, -1, 0,
         -w, -h,  d,    0, -1, 0,
         -w, -h, -d,    0, -1, 0
    };
    return vertices;
}

std::vector<float> get_algae_vertices(float width, float height) {
    float w = width / 2;
    std::vector<float> vertices = {

        -w, 0.0f, 0.0f,    0, 0, 1,
         w, -height, 0.0f,  0, 0, 1,
        -w, -height, 0.0f,  0, 0, 1,

        -w, 0.0f, 0.0f,    0, 0, 1,
         w, 0.0f, 0.0f,    0, 0, 1,
         w, -height, 0.0f,  0, 0, 1
    };
    return vertices;
}

// ----------- Generowanie terenu -------------
void generateTerrainMesh(float size, int resolution, float amplitude,
    std::vector<float>& vertices, std::vector<unsigned int>& indices)
{
    float step = size / (resolution - 1);
    float baseHeight = -size / 2.0f;
    gTerrainSize = size;
    gTerrainResolution = resolution;
    gTerrainAmplitude = amplitude;
    gTerrainBase = baseHeight;

    float offsetX = static_cast<float>(rand()) / RAND_MAX * 10.0f;
    float offsetZ = static_cast<float>(rand()) / RAND_MAX * 10.0f;
    gTerrainOffsetX = offsetX;
    gTerrainOffsetZ = offsetZ;

    std::vector<std::vector<float>> heights(resolution, std::vector<float>(resolution, 0.0f));
    for (int j = 0; j < resolution; j++) {
        for (int i = 0; i < resolution; i++) {
            float x = -size / 2.0f + i * step;
            float z = -size / 2.0f + j * step;
            if (i == 0 || i == resolution - 1 || j == 0 || j == resolution - 1)
                heights[j][i] = baseHeight;
            else {
                float h = amplitude * terrainNoise(x + offsetX, z + offsetZ);
                heights[j][i] = baseHeight + h;
            }
        }
    }
    for (int j = 0; j < resolution; j++) {
        for (int i = 0; i < resolution; i++) {
            float x = -size / 2.0f + i * step;
            float z = -size / 2.0f + j * step;
            float y = heights[j][i];
            float u = float(i) / (resolution - 1);
            float v = float(j) / (resolution - 1);
            float hL = (i > 0) ? heights[j][i - 1] : y;
            float hR = (i < resolution - 1) ? heights[j][i + 1] : y;
            float hD = (j > 0) ? heights[j - 1][i] : y;
            float hU = (j < resolution - 1) ? heights[j + 1][i] : y;
            float dheight_dx = (hR - hL) / (2 * step);
            float dheight_dz = (hU - hD) / (2 * step);
            glm::vec3 n = glm::normalize(glm::vec3(-dheight_dx, 1.0f, -dheight_dz));
            glm::vec3 tangent = glm::normalize(glm::vec3(1.0f, dheight_dx, 0.0f));
            glm::vec3 bitangent = glm::normalize(glm::cross(n, tangent));
            vertices.push_back(x);
            vertices.push_back(y);
            vertices.push_back(z);
            vertices.push_back(n.x);
            vertices.push_back(n.y);
            vertices.push_back(n.z);
            vertices.push_back(u);
            vertices.push_back(v);
            vertices.push_back(tangent.x);
            vertices.push_back(tangent.y);
            vertices.push_back(tangent.z);
            vertices.push_back(bitangent.x);
            vertices.push_back(bitangent.y);
            vertices.push_back(bitangent.z);
        }
    }
    for (int j = 0; j < resolution - 1; j++) {
        for (int i = 0; i < resolution - 1; i++) {
            int topLeft = j * resolution + i;
            int topRight = topLeft + 1;
            int bottomLeft = (j + 1) * resolution + i;
            int bottomRight = bottomLeft + 1;
            indices.push_back(topLeft);
            indices.push_back(bottomLeft);
            indices.push_back(topRight);
            indices.push_back(topRight);
            indices.push_back(bottomLeft);
            indices.push_back(bottomRight);
        }
    }
}


void handleCollisions(float deltaTime) {
    for (auto& fish : fishes) {
        glm::mat4 fishModel = glm::translate(glm::mat4(1.0f), fish.position) *
            glm::toMat4(fish.orientation);
        AABB fishAABB = transformAABB(fishModel, fishLocalMin, fishLocalMax);
        for (const auto& obs : obstacles) {
            glm::mat4 obsModel = glm::translate(glm::mat4(1.0f), obs.position);
            obsModel = glm::rotate(obsModel, glm::radians(obs.rotation), glm::vec3(0, 1, 0));
            obsModel = glm::scale(obsModel, obs.scale);
            AABB obsAABB;
            if (obs.type == CACTUS)
                obsAABB = transformAABB(obsModel, cactusLocalMin, cactusLocalMax);
            else if (obs.type == ALGAE)
                obsAABB = transformAABB(obsModel, algaeLocalMin, algaeLocalMax);
            if (aabbIntersect(fishAABB, obsAABB)) {
                fish.velocity = -fish.velocity;
                fish.position += fish.velocity * deltaTime;
            }
        }
    }
}

void handleFishCollisions(float deltaTime) {
    for (size_t i = 0; i < fishes.size(); i++) {
        glm::mat4 fishModel_i = glm::translate(glm::mat4(1.0f), fishes[i].position) *
            glm::toMat4(fishes[i].orientation);
        AABB fishAABB_i = transformAABB(fishModel_i, fishLocalMin, fishLocalMax);
        for (size_t j = i + 1; j < fishes.size(); j++) {
            glm::mat4 fishModel_j = glm::translate(glm::mat4(1.0f), fishes[j].position) *
                glm::toMat4(fishes[j].orientation);
            AABB fishAABB_j = transformAABB(fishModel_j, fishLocalMin, fishLocalMax);
            if (aabbIntersect(fishAABB_i, fishAABB_j)) {
                fishes[i].velocity = -fishes[i].velocity;
                fishes[i].position += fishes[i].velocity * deltaTime;
                fishes[j].velocity = -fishes[j].velocity;
                fishes[j].position += fishes[j].velocity * deltaTime;
            }
        }
    }
}

void initFishes() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-10.0, 10.0);
    for (int i = 0; i < NUM_FISHES; ++i) {
        fishes.emplace_back(glm::vec3(dis(gen), dis(gen), dis(gen)));
    }
}

void updateFishes(float deltaTime) {
    for (auto& fish : fishes) {
        fish.update(fishes, deltaTime, player.position, player.attractionWeight);
    }
}

void updateDeltaTime(float time) {
    if (lastTime < 0) {
        lastTime = time;
        return;
    }
    deltaTime = time - lastTime;
    if (deltaTime > 0.1f)
        deltaTime = 0.1f;
    lastTime = time;
}

glm::mat4 createCameraMatrix() {
    return glm::lookAt(cameraPos, cameraPos + cameraDir, up);
}

glm::mat4 createPerspectiveMatrix() {
    float fov = glm::radians(60.0f);
    float near = 0.1f;
    float far = 100.0f;
    return glm::perspective(fov, aspectRatio, near, far);
}

// ----------- Renderowanie -------------
void renderFishes(GLuint shaderProgram) {
    glUniform1i(glGetUniformLocation(shaderProgram, "useTexture"), 0);
    glBindVertexArray(fishVAO);
    for (const auto& fish : fishes) {
        glm::mat4 modelMatrix = glm::translate(glm::mat4(1.0f), fish.position) * glm::toMat4(fish.orientation) * glm::eulerAngleX(glm::radians(-90.f));
        GLint modelLoc = glGetUniformLocation(shaderProgram, "model");
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(modelMatrix));
        GLint objectColorLoc = glGetUniformLocation(shaderProgram, "objectColor");
        glUniform3fv(objectColorLoc, 1, glm::value_ptr(fish.color));
        glDrawArrays(GL_TRIANGLES, 0, 18);
    }
    glBindVertexArray(0);
}

void renderObstacles(GLuint shaderProgram) {
    glUniform1i(glGetUniformLocation(shaderProgram, "useTexture"), 0);
    glUniform3f(glGetUniformLocation(shaderProgram, "objectColor"), 0.0f, 0.8f, 0.0f);
    for (const auto& obs : obstacles) {
        glm::mat4 model = glm::translate(glm::mat4(1.0f), obs.position);
        model = glm::rotate(model, glm::radians(obs.rotation), glm::vec3(0, 1, 0));
        model = glm::scale(model, obs.scale);
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        if (obs.type == CACTUS) {
            glBindVertexArray(cactusVAO);
            glDrawArrays(GL_TRIANGLES, 0, 36);
            glBindVertexArray(0);
        }
        else if (obs.type == ALGAE) {
            glBindVertexArray(algaeVAO);
            glDrawArrays(GL_TRIANGLES, 0, 6);
            glBindVertexArray(0);
        }
    }
}

void renderScene() {
    calculateLightSpaceMatrix();
    float time = glfwGetTime();
    updateDeltaTime(time);

    player.position = cameraPos + cameraDir * 3.0f;

    updateFishes(deltaTime);
    handleCollisions(deltaTime);
    handleFishCollisions(deltaTime);

    // rybki
    glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glClear(GL_DEPTH_BUFFER_BIT);
    glUseProgram(programDepth);
    GLint lightSpaceMatrixLoc = glGetUniformLocation(programDepth, "lightSpaceMatrix");
    glUniformMatrix4fv(lightSpaceMatrixLoc, 1, GL_FALSE, glm::value_ptr(lightSpaceMatrix));
    if (shadowMappingEnabled) {
        renderFishes(programDepth);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glViewport(0, 0, 1000, 1000);
    glClearColor(0.0f, 0.2f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram(program);

    glUniform1i(glGetUniformLocation(program, "useShadowMapping"), 1 );
    glUniform1i(glGetUniformLocation(program, "useNormalMapping"), normalMappingEnabled ? 1 : 0);

    setLightUniforms(program, cameraPos);
    lightSpaceMatrixLoc = glGetUniformLocation(program, "lightSpaceMatrix");
    glUniformMatrix4fv(lightSpaceMatrixLoc, 1, GL_FALSE, glm::value_ptr(lightSpaceMatrix));

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    glUniform1i(glGetUniformLocation(program, "shadowMap"), 0);

    glm::mat4 view = createCameraMatrix();
    glm::mat4 projection = createPerspectiveMatrix();
    glUniformMatrix4fv(glGetUniformLocation(program, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

    renderFishes(program);

    // granice akwarium
    glUniform1i(glGetUniformLocation(program, "useTexture"), 0);
    {
        glm::mat4 model = glm::mat4(1.0f);
        glUniformMatrix4fv(glGetUniformLocation(program, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glUniform3f(glGetUniformLocation(program, "objectColor"), 1.0f, 1.0f, 1.0f);
        glBindVertexArray(borderVAO);
        glLineWidth(3.0f);
        glDrawArrays(GL_LINES, 0, 24);
        glLineWidth(1.0f);
        glBindVertexArray(0);
    }

    // teren
    glBindVertexArray(floorVAO);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, floorDiffuseTexture);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, floorNormalTexture);
    glUniform1i(glGetUniformLocation(program, "diffuseMap"), 1);
    glUniform1i(glGetUniformLocation(program, "normalMap"), 2);
    glUniform1i(glGetUniformLocation(program, "useTexture"), 1);
    glDrawElements(GL_TRIANGLES, floorIndexCount, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    renderObstacles(program);

    // gracz
    {
        
        glm::mat4 viewMat = createCameraMatrix();
        glm::mat3 viewRot = glm::mat3(viewMat);
        glm::mat3 invRot = glm::transpose(viewRot);
        glm::mat4 invRotation = glm::mat4(invRot);

        glm::mat4 submarineModel = glm::translate(glm::mat4(1.0f), cameraPos + cameraDir * 3.0f);
        submarineModel *= invRotation; 
        submarineModel = glm::scale(submarineModel, glm::vec3(0.5f));

        glUniformMatrix4fv(glGetUniformLocation(program, "model"), 1, GL_FALSE, glm::value_ptr(submarineModel));
        glUniform3f(glGetUniformLocation(program, "objectColor"), 1.0f, 1.0f, 0.0f);
        glBindVertexArray(submarineVAO);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        glBindVertexArray(0);
    }

    glfwSwapBuffers(glfwGetCurrentContext());
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    aspectRatio = width / float(height);
    glViewport(0, 0, width, height);
}

// ----------- Klawiatura -------------
void processInput(GLFWwindow* window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        cameraPos += cameraDir * moveSpeed * deltaTime;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        cameraPos -= cameraDir * moveSpeed * deltaTime;

    glm::vec3 right = glm::normalize(glm::cross(cameraDir, up));
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        cameraPos -= right * moveSpeed * deltaTime;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        cameraPos += right * moveSpeed * deltaTime;

    if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS) {
        separationWeight += 0.1f;
        if (separationWeight > 10.f) {
            separationWeight = 10.0f;
        }
    }
    if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS) {
        separationWeight -= 0.1f;
        if (separationWeight < 1.5f) {
            separationWeight = 1.5f;
        }
    }
    if (glfwGetKey(window, GLFW_KEY_Y) == GLFW_PRESS) {
        alignmentWeight += 0.1f;
        if (alignmentWeight > 10.f) {
            alignmentWeight = 10.0f;
        }
    }
    if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS) {
        alignmentWeight -= 0.1f;
        if (alignmentWeight < 0.f) {
            alignmentWeight = 0.0f;
        }
    }
    if (glfwGetKey(window, GLFW_KEY_U) == GLFW_PRESS) {
        cohesionWeight += 0.1f;
        if (cohesionWeight > 10.f) {
            cohesionWeight = 10.0f;
        }
    }
    if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS) {
        cohesionWeight -= 0.1f;
        if (cohesionWeight < 0.f) {
            cohesionWeight = 0.0f;
        }
    }

    if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS) {
        player.attractionWeight += 0.1f;
        if (player.attractionWeight > 3.f) {
            player.attractionWeight = 3.0f;
        }
    }
    if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS) {
        player.attractionWeight -= 0.1f;
        if (player.attractionWeight < 0.f) {
            player.attractionWeight = 0.0f;
        }
    }

    if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS && !zKeyPressed) {
        shadowMappingEnabled = !shadowMappingEnabled;
        zKeyPressed = true;
    }
    if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_RELEASE) {
        zKeyPressed = false;
    }
}

bool running = true;
void print_info() {
    while (running) {
        system("cls");
        std::cout << "separationWeight: " << separationWeight << std::endl;
        std::cout << "alignmentWeight: " << alignmentWeight << std::endl;
        std::cout << "cohesionWeight: " << cohesionWeight << std::endl;
        std::cout << "playerAttractionWeight: " << player.attractionWeight << std::endl;
        std::cout << "shadowMappingEnabled: " << shadowMappingEnabled << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void renderLoop(GLFWwindow* window) {
    std::thread info_thread(print_info);
    while (!glfwWindowShouldClose(window)) {
        processInput(window);
        renderScene();
        glfwPollEvents();
    }
    running = false;
    info_thread.join();
}

void init(GLFWwindow* window) {
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glEnable(GL_DEPTH_TEST);

    program = shaderLoader.CreateProgram("../shaders/shader_5_1.vert", "../shaders/shader_5_1.frag");
    programDepth = shaderLoader.CreateProgram("../shaders/depthShader.vert", "../shaders/depthShader.frag");

    // rybki
    {
        std::vector<float> fishVertices = get_pyramid_vertices(0.5f, 0.5f, 0.5f);
        glGenVertexArrays(1, &fishVAO);
        glGenBuffers(1, &fishVBO);
        glBindVertexArray(fishVAO);
        glBindBuffer(GL_ARRAY_BUFFER, fishVBO);
        glBufferData(GL_ARRAY_BUFFER, fishVertices.size() * sizeof(float), fishVertices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }
    // granice akwarium
    {
        auto get_cube_lines_corrected = [](float size) -> std::vector<float> {
            float half = size / 2.0f;
            std::vector<glm::vec3> corners = {
                glm::vec3(-half, -half, -half),
                glm::vec3(half, -half, -half),
                glm::vec3(half, -half,  half),
                glm::vec3(-half, -half,  half),
                glm::vec3(-half,  half, -half),
                glm::vec3(half,  half, -half),
                glm::vec3(half,  half,  half),
                glm::vec3(-half,  half,  half)
            };
            std::vector<int> indices = {
                0,1, 1,2, 2,3, 3,0,
                4,5, 5,6, 6,7, 7,4,
                0,4, 1,5, 2,6, 3,7
            };
            std::vector<float> lines;
            for (int idx : indices) {
                lines.push_back(corners[idx].x);
                lines.push_back(corners[idx].y);
                lines.push_back(corners[idx].z);
                lines.push_back(0.0f);
                lines.push_back(0.0f);
                lines.push_back(0.0f);
            }
            return lines;
            };
        std::vector<float> borderVertices = get_cube_lines_corrected(20.0f);
        glGenVertexArrays(1, &borderVAO);
        glGenBuffers(1, &borderVBO);
        glBindVertexArray(borderVAO);
        glBindBuffer(GL_ARRAY_BUFFER, borderVBO);
        glBufferData(GL_ARRAY_BUFFER, borderVertices.size() * sizeof(float), borderVertices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }
    // teren
    {
        float terrainSize = 20.0f;
        int resolution = 50;
        float amplitude = 1.0f;
        std::vector<float> vertices;
        std::vector<unsigned int> indices;
        generateTerrainMesh(terrainSize, resolution, amplitude, vertices, indices);
        floorIndexCount = indices.size();
        glGenVertexArrays(1, &floorVAO);
        glGenBuffers(1, &floorVBO);
        glGenBuffers(1, &floorEBO);
        glBindVertexArray(floorVAO);
        glBindBuffer(GL_ARRAY_BUFFER, floorVBO);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, floorEBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(6 * sizeof(float)));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(8 * sizeof(float)));
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(11 * sizeof(float)));
        glEnableVertexAttribArray(4);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }
    // kaktusy
    {
        std::vector<float> cactusVertices = get_cube_vertices(1.0f, 4.0f, 1.0f);
        glGenVertexArrays(1, &cactusVAO);
        glGenBuffers(1, &cactusVBO);
        glBindVertexArray(cactusVAO);
        glBindBuffer(GL_ARRAY_BUFFER, cactusVBO);
        glBufferData(GL_ARRAY_BUFFER, cactusVertices.size() * sizeof(float), cactusVertices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
    }
    // glony
    {
        std::vector<float> algaeVertices = get_algae_vertices(0.5f, 6.0f);
        glGenVertexArrays(1, &algaeVAO);
        glGenBuffers(1, &algaeVBO);
        glBindVertexArray(algaeVAO);
        glBindBuffer(GL_ARRAY_BUFFER, algaeVBO);
        glBufferData(GL_ARRAY_BUFFER, algaeVertices.size() * sizeof(float), algaeVertices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
    }
    // lodz podwodna
    {
        std::vector<float> submarineVertices = get_cube_vertices(1.0f, 1.0f, 1.0f);
        glGenVertexArrays(1, &submarineVAO);
        glGenBuffers(1, &submarineVBO);
        glBindVertexArray(submarineVAO);
        glBindBuffer(GL_ARRAY_BUFFER, submarineVBO);
        glBufferData(GL_ARRAY_BUFFER, submarineVertices.size() * sizeof(float), submarineVertices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
    }

    initFishes();
    initShadowMap();
    {
        int numCactus = 5, numAlgae = 5;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> disXZ(-gTerrainSize / 2.0f + 2.0f, gTerrainSize / 2.0f - 2.0f);
        for (int i = 0; i < numCactus; i++) {
            float x = disXZ(gen);
            float z = disXZ(gen);
            float y = gTerrainBase + gTerrainAmplitude * terrainNoise(x + gTerrainOffsetX, z + gTerrainOffsetZ);
            Obstacle obs;
            obs.type = CACTUS;
            obs.position = glm::vec3(x, y + 1.8f, z);
            obs.scale = glm::vec3(1.0f);
            obs.rotation = static_cast<float>(rand() % 360);
            obstacles.push_back(obs);
        }
        std::uniform_real_distribution<float> disXZ2(-gTerrainSize / 2.0f + 2.0f, gTerrainSize / 2.0f - 2.0f);
        for (int i = 0; i < numAlgae; i++) {
            float x = disXZ2(gen);
            float z = disXZ2(gen);
            float y = gTerrainSize / 2.0f;
            Obstacle obs;
            obs.type = ALGAE;
            obs.position = glm::vec3(x, y, z);
            obs.scale = glm::vec3(1.0f);
            obs.rotation = static_cast<float>(rand() % 360);
            obstacles.push_back(obs);
        }
    }
    player.attractionWeight = 0.2f;

    floorDiffuseTexture = loadTexture("../textures/sand.jpg");
    floorNormalTexture = loadTexture("../textures/sand_normal.jpg");
}

void shutdown(GLFWwindow* window) {
    glDeleteVertexArrays(1, &fishVAO);
    glDeleteBuffers(1, &fishVBO);
    glDeleteVertexArrays(1, &borderVAO);
    glDeleteBuffers(1, &borderVBO);
    glDeleteVertexArrays(1, &floorVAO);
    glDeleteBuffers(1, &floorVBO);
    glDeleteBuffers(1, &floorEBO);
    glDeleteVertexArrays(1, &cactusVAO);
    glDeleteBuffers(1, &cactusVBO);
    glDeleteVertexArrays(1, &algaeVAO);
    glDeleteBuffers(1, &algaeVBO);
    glDeleteVertexArrays(1, &submarineVAO);
    glDeleteBuffers(1, &submarineVBO);
    glDeleteTextures(1, &floorDiffuseTexture);
    glDeleteTextures(1, &floorNormalTexture);
    shaderLoader.DeleteProgram(program);
}

int main(int argc, char** argv) {
    if (!glfwInit()) {
        std::cout << "GLFW initialization failed" << std::endl;
        return -1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    GLFWwindow* window = glfwCreateWindow(1000, 1000, "Aquarium", NULL, NULL);
    if (window == NULL) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glewInit();
    glViewport(0, 0, 1000, 1000);

    init(window);
    renderLoop(window);
    shutdown(window);
    glfwTerminate();
    return 0;
}
