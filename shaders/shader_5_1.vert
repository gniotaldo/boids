#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec3 vertexNormal;
layout(location = 2) in vec2 vertexTexCoords;    
layout(location = 3) in vec3 vertexTangent;
layout(location = 4) in vec3 vertexBitangent;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform mat4 lightSpaceMatrix;
uniform vec3 lightPos;    
uniform vec3 viewPos;     

out vec3 fragPos;
out vec3 normal;
out vec4 fragPosLightSpace;

out vec2 TexCoords;
out vec3 TangentLightPos;
out vec3 TangentViewPos;
out vec3 TangentFragPos;

void main()
{
    fragPos = vec3(model * vec4(vertexPosition, 1.0));
    mat3 normalMatrix = transpose(inverse(mat3(model)));
    normal = normalMatrix * vertexNormal;
    fragPosLightSpace = lightSpaceMatrix * vec4(fragPos, 1.0);
    
    TexCoords = vertexTexCoords;
    
    vec3 T = normalize(normalMatrix * vertexTangent);
    vec3 N = normalize(normalMatrix * vertexNormal);

    T = normalize(T - dot(T, N) * N);
    vec3 B = cross(N, T);
    
    mat3 TBN = transpose(mat3(T, B, N));

    TangentLightPos = TBN * lightPos;
    TangentViewPos = TBN * viewPos;
    TangentFragPos = TBN * fragPos;
    
    gl_Position = projection * view * model * vec4(vertexPosition, 1.0);
}