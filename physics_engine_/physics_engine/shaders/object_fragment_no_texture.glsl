#version 330 core
in vec3 FragPos;
out vec4 FragColor;

uniform vec3 objectColor;
uniform vec3 viewPos;

void main()
{
  FragColor = vec4(objectColor, 1.0);
}