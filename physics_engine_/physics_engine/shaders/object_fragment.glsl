#version 330 core
in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoord;

out vec4 FragColor;

uniform vec3 objectColor;
uniform vec3 viewPos;
uniform sampler2D texture1;

void main()
{
  FragColor = texture(texture1, TexCoord);
}