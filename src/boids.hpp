#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <vector>
#include <iostream>
#include <cmath>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/euler_angles.hpp>

#include "Shader_Loader.h"
#include "Camera.h"

#include <string>

GLuint program;
Core::Shader_Loader shaderLoader;

float moveSpeed = 10.f;

GLuint VAO, VBO;
float aspectRatio = 1.f;

float lastTime = -1.f;
float deltaTime = 0.f;

std::vector<float> get_cuboid_vertexes(float width, float height, float depth) {
    float x = width / 2.0f;
    float z = depth / 2.0f;

    std::vector<float> box = {
        x, 0.0f, z, 1.0f,
        x, height, z, 1.0f,
        -x, 0.0f, z, 1.0f,

        x, height, z, 1.0f,
        -x, height, z, 1.0f,
        -x, 0.0f, z, 1.0f,

        x, 0.0f, -z, 1.0f,
        -x, 0.0f, -z, 1.0f,
        x, height, -z, 1.0f,

        x, height, -z, 1.0f,
        -x, 0.0f, -z, 1.0f,
        -x, height, -z, 1.0f,

        -x, 0.0f, z, 1.0f,
        -x, height, z, 1.0f,
        -x, height, -z, 1.0f,

        -x, 0.0f, z, 1.0f,
        -x, height, -z, 1.0f,
        -x, 0.0f, -z, 1.0f,

        x, 0.0f, z, 1.0f,
        x, height, -z, 1.0f,
        x, height, z, 1.0f,

        x, 0.0f, z, 1.0f,
        x, 0.0f, -z, 1.0f,
        x, height, -z, 1.0f,

        x, 0.0f, -z, 1.0f,
        x, 0.0f, z, 1.0f,
        -x, 0.0f, z, 1.0f,

        x, 0.0f, -z, 1.0f,
        -x, 0.0f, z, 1.0f,
        -x, 0.0f, -z, 1.0f,

        x, height, -z, 1.0f,
        -x, height, z, 1.0f,
        x, height, z, 1.0f,

        x, height, -z, 1.0f,
        -x, height, -z, 1.0f,
        -x, height, z, 1.0f
    };

    return box;
}

void updateDeltaTime(float time) {
    if (lastTime < 0) {
        lastTime = time;
        return;
    }
    deltaTime = time - lastTime;
    if (deltaTime > 0.1) deltaTime = 0.1;
    lastTime = time;
}


glm::vec3 cameraPos = glm::vec3(0.f, 1.f, 5.f);  // Pozycja kamery
glm::vec3 cameraDir = glm::vec3(0.f, 0.f, -1.f); // Kierunek patrzenia kamery (wzdłuż osi Z)
glm::vec3 up = glm::vec3(0.f, 1.f, 0.f);         // Wektor w górę (poziom kamery)


glm::mat4 createCameraMatrix() {
    return glm::lookAt(cameraPos, cameraPos + cameraDir, up); // Kamera patrzy w kierunku cameraDir
}



glm::mat4 createPerspectiveMatrix() {
    float fov = glm::radians(60.0f);
    float near = 0.1f;
    float far = 100.0f;
    return glm::perspective(fov, aspectRatio, near, far);
}

void renderScene(GLFWwindow* window) {
    glClearColor(0.0f, 0.2f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    float time = glfwGetTime();
    updateDeltaTime(time);

    glUseProgram(program);

    glm::mat4 view = createCameraMatrix();
    glm::mat4 projection = createPerspectiveMatrix();
    glm::mat4 VP = projection * view;

    // Tworzenie transformacji dla sześcianu na środku
    glm::mat4 modelMatrix = glm::translate(glm::mat4(1.f), glm::vec3(0.f, 0.f, 0.f));
    glm::mat4 transformation = VP * modelMatrix;

    glUniformMatrix4fv(glGetUniformLocation(program, "transformation"), 1, GL_FALSE, &transformation[0][0]);

    glUniform3fv(glGetUniformLocation(program, "color"), 1, glm::value_ptr(glm::vec3(1.0f, 0.0f, 0.0f))); // Czerwony kolor sześcianu

    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, 36);  // Rysowanie sześcianu
    glfwSwapBuffers(window);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    aspectRatio = width / float(height);
    glViewport(0, 0, width, height);
}

void init(GLFWwindow* window) {
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    glEnable(GL_DEPTH_TEST);
    program = shaderLoader.CreateProgram((char*)"../shaders/shader_5_1.vert", (char*)"../shaders/shader_5_1.frag");

    // Przygotowanie wierzchołków sześcianu
    std::vector<float> vertices = get_cuboid_vertexes(1.0f, 1.0f, 1.0f);

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void shutdown(GLFWwindow* window) {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    shaderLoader.DeleteProgram(program);
}

void processInput(GLFWwindow* window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        cameraPos += glm::vec3(0.f, 0.f, -1.f) * moveSpeed * deltaTime;  // Ruch w przód
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        cameraPos += glm::vec3(0.f, 0.f, 1.f) * moveSpeed * deltaTime;  // Ruch w tył
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        cameraPos += glm::vec3(-1.f, 0.f, 0.f) * moveSpeed * deltaTime;  // Ruch w lewo
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        cameraPos += glm::vec3(1.f, 0.f, 0.f) * moveSpeed * deltaTime;  // Ruch w prawo
    }
}


void renderLoop(GLFWwindow* window) {
    while (!glfwWindowShouldClose(window)) {
        processInput(window);
        renderScene(window);
        glfwPollEvents();
    }
}