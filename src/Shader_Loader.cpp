#include "Shader_Loader.h"
#include <fstream>
#include <sstream>
#include <iostream>

namespace Core {

    GLuint Shader_Loader::CreateProgram(const char* vertex_path, const char* fragment_path) {
        std::string vertexCode;
        std::string fragmentCode;
        std::ifstream vShaderFile;
        std::ifstream fShaderFile;

        // Ustawienie wyj¹tków dla strumieni
        vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);

        try {
            // Otwieramy pliki shaderów
            vShaderFile.open(vertex_path);
            fShaderFile.open(fragment_path);
            std::stringstream vShaderStream, fShaderStream;
            // Wczytujemy zawartoœæ plików do strumieni
            vShaderStream << vShaderFile.rdbuf();
            fShaderStream << fShaderFile.rdbuf();
            // Zamykamy pliki
            vShaderFile.close();
            fShaderFile.close();
            // Konwertujemy strumienie na stringi
            vertexCode = vShaderStream.str();
            fragmentCode = fShaderStream.str();
        }
        catch (std::ifstream::failure& e) {
            std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ: " << e.what() << std::endl;
        }

        const char* vShaderCode = vertexCode.c_str();
        const char* fShaderCode = fragmentCode.c_str();

        GLuint vertex, fragment;
        GLint success;
        GLchar infoLog[512];

        // Kompilacja vertex shader
        vertex = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertex, 1, &vShaderCode, NULL);
        glCompileShader(vertex);
        glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(vertex, 512, NULL, infoLog);
            std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
        }

        // Kompilacja fragment shader
        fragment = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragment, 1, &fShaderCode, NULL);
        glCompileShader(fragment);
        glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(fragment, 512, NULL, infoLog);
            std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
        }

        // Linkowanie programu shaderowego
        GLuint shaderProgram = glCreateProgram();
        glAttachShader(shaderProgram, vertex);
        glAttachShader(shaderProgram, fragment);
        glLinkProgram(shaderProgram);
        glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
            std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
        }

        // Usuwamy shadery, gdy¿ ju¿ nie s¹ potrzebne
        glDeleteShader(vertex);
        glDeleteShader(fragment);

        return shaderProgram;
    }

    void Shader_Loader::DeleteProgram(GLuint program) {
        glDeleteProgram(program);
    }

} // namespace Core
