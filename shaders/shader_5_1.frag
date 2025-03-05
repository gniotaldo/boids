#version 430 core
struct Light {
    vec3 position;
    vec3 color;
    float intensity;
};

uniform Light light;
uniform vec3 viewPos;
uniform vec3 objectColor;
uniform sampler2D shadowMap;
uniform sampler2D diffuseMap;    
uniform sampler2D normalMap;
uniform bool useTexture;         
uniform mat4 lightSpaceMatrix;

in vec3 fragPos;
in vec3 normal;
in vec4 fragPosLightSpace;
in vec2 TexCoords;              
in vec3 TangentLightPos;        
in vec3 TangentViewPos;
in vec3 TangentFragPos;

out vec4 outColor;

float ShadowCalculation(vec4 fragPosLightSpace)
{
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    
    projCoords = projCoords * 0.5 + 0.5;
    
    if(projCoords.z > 1.0)
        return 0.0;
        
    float currentDepth = projCoords.z;
    float closestDepth = texture(shadowMap, projCoords.xy).r;
    
    float bias = 0.0005;
    float shadow = currentDepth - bias > closestDepth ? 1.0 : 0.0;
    
    return shadow;
}

void main()
{
    vec3 norm;
    vec3 lightDir;
    vec3 viewDir;
    
    if (useTexture) {
        norm = normalize(texture(normalMap, TexCoords).rgb * 2.0 - 1.0);
        lightDir = normalize(TangentLightPos - TangentFragPos);
        viewDir = normalize(TangentViewPos - TangentFragPos);
    } else {
        norm = normalize(normal);
        lightDir = normalize(light.position - fragPos);
        viewDir = normalize(viewPos - fragPos);
    }

    float distance = length(light.position - fragPos);
    float attenuation = 1.0 / (1.0 + 0.09 * distance + 0.032 * (distance * distance));

    // ambient
    float ambientStrength = 0.3;
    vec3 ambient = ambientStrength * light.color;
    
    // diffuse
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * light.color * light.intensity * attenuation;
    
    // specular
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
    vec3 specular = spec * light.color * attenuation;
    
    float shadow = ShadowCalculation(fragPosLightSpace);
    
    vec3 baseColor;
    if (useTexture) {
        baseColor = texture(diffuseMap, TexCoords).rgb;
    } else {
        baseColor = objectColor;
    }
    
    vec3 result = (ambient + (1.0 - shadow) * (diffuse + specular)) * baseColor;
    outColor = vec4(result, 1.0);
}