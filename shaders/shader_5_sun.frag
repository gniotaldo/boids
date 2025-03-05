#version 430 core

uniform vec3 color;

in vec3 interpNormal;

out vec4 out_color;

void main()
{
	out_color = vec4(color, 1.0);
}
