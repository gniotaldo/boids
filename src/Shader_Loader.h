#ifndef SHADER_LOADER_H
#define SHADER_LOADER_H

#include <GL/glew.h>
#include <string>

namespace Core {
    class Shader_Loader {
    public:
        // Kompilacja i linkowanie shaderów – implementacja w Shader_Loader.cpp
        GLuint CreateProgram(const char* vertex_path, const char* fragment_path);
        // Usuwanie programu shaderowego
        void DeleteProgram(GLuint program);
    };
}

#endif // SHADER_LOADER_H
